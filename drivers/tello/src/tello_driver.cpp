#include "drivers/tello/tello_driver.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "common/logging.hpp"
#include "common/net/udp_socket.hpp"
#include "tello_state_parser.hpp"

namespace drivers::tello {

struct TelloDriver::Impl {
    common::net::UdpSocket command_socket;
    common::net::UdpSocket state_socket;
};

TelloDriver::TelloDriver(TelloConnectionConfig config, core::EventBus& bus)
    : config_(std::move(config)), bus_(bus), impl_(std::make_unique<Impl>()) {}

TelloDriver::~TelloDriver() { disconnect(); }

bool TelloDriver::connect() {
    if (!impl_->command_socket.bind(config_.local_command_port)) {
        common::logError("TelloDriver", "failed to bind command socket");
        return false;
    }
    if (!impl_->state_socket.bind(config_.state_port)) {
        common::logError("TelloDriver", "failed to bind state socket");
        return false;
    }
    if (!sendCommandAndWaitOk("command", config_.command_timeout_ms, config_.command_retries)) {
        common::logError("TelloDriver", "drone did not acknowledge SDK mode entry");
        return false;
    }

    connected_ = true;
    state_thread_ = std::jthread([this](std::stop_token token) { stateReceiveLoop(token); });
    keep_alive_thread_ = std::jthread([this](std::stop_token token) { keepAliveLoop(token); });
    common::logInfo("TelloDriver", "connected, drone in SDK mode");
    return true;
}

void TelloDriver::disconnect() {
    if (!connected_) return;
    connected_ = false;

    if (state_thread_.joinable()) {
        state_thread_.request_stop();
        state_thread_.join();
    }
    if (keep_alive_thread_.joinable()) {
        keep_alive_thread_.request_stop();
        keep_alive_thread_.join();
    }
    impl_->command_socket.close();
    impl_->state_socket.close();
}

bool TelloDriver::sendCommandAndWaitOk(const std::string& command, int timeoutMs, int attempts) {
    std::lock_guard lock(command_mutex_);
    // The command channel has no request id, so a reply that arrives late -
    // a slow takeoff's "ok", say - would be read as the answer to whatever
    // is asked next. Clear the backlog first.
    if (const std::size_t stale = impl_->command_socket.drain(); stale > 0) {
        common::logDebug("TelloDriver",
                          "discarded " + std::to_string(stale) + " stale reply datagram(s)");
    }
    for (int attempt = 0; attempt < attempts; ++attempt) {
        if (!impl_->command_socket.sendTo(config_.ip, config_.command_port, command)) {
            continue;
        }
        last_command_sent_at_.store(std::chrono::steady_clock::now(), std::memory_order_relaxed);
        if (auto response = impl_->command_socket.receive(timeoutMs)) {
            common::logDebug("TelloDriver", "cmd '" + command + "' -> '" + *response + "'");
            if (response->find("ok") != std::string::npos) {
                return true;
            }
            // A reply that is not "ok" is the drone REFUSING, and it says why
            // ("error Motor stop", "error Not joystick", "error Auto land").
            // Logging only "no ack" here threw that away and left a refusal
            // looking identical to a dead link - completely different problems.
            common::logWarn("TelloDriver",
                             "drone refused '" + command + "': \"" + *response + "\"");
        } else {
            common::logWarn("TelloDriver",
                             "no reply to '" + command + "' within " +
                                 std::to_string(timeoutMs) + "ms");
        }
    }
    return false;
}

bool TelloDriver::takeoff() {
    // ONE attempt, never a retry. Takeoff is a physical action, not an
    // idempotent query: if the first one was received, a retry is at best
    // refused and at worst fights the manoeuvre in progress.
    if (sendCommandAndWaitOk("takeoff", config_.takeoff_timeout_ms, /*attempts=*/1)) {
        return true;
    }

    // No ack is not the same as no takeoff. Ask the airframe instead of the
    // command channel - being wrong here means either refusing to fly a
    // healthy drone, or walking away from one that is already hovering.
    for (int waited = 0; waited < 30; ++waited) {
        if (airborneByTelemetry().value_or(false)) {
            common::logWarn("TelloDriver",
                             "takeoff was not acknowledged, but telemetry says the drone is "
                             "airborne - continuing");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

bool TelloDriver::land() {
    // Retried, unlike takeoff: a drone still in the air is the one situation
    // worth being insistent about, and a repeated `land` on the ground is
    // harmless.
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (sendCommandAndWaitOk("land", config_.land_timeout_ms, /*attempts=*/1)) return true;

        const std::optional<bool> airborne = airborneByTelemetry();
        if (airborne.has_value() && !*airborne) {
            common::logWarn("TelloDriver",
                             "no ack for 'land', but telemetry says the drone is on the ground");
            return true;
        }
        common::logWarn("TelloDriver",
                         airborne.has_value()
                             ? "still airborne after 'land' - trying again"
                             : "no ack for 'land' and no telemetry to check against - trying "
                               "again rather than assuming it landed");
    }
    return false;
}

std::optional<bool> TelloDriver::airborneByTelemetry() const {
    std::lock_guard lock(state_mutex_);
    if (!have_state_) return std::nullopt;
    // Stale telemetry is no better than none: the drone may have taken off
    // since the last packet we saw.
    if (std::chrono::steady_clock::now() - last_state_.received_at > std::chrono::seconds(3)) {
        return std::nullopt;
    }
    // Height is centimetres above the takeoff point; a few centimetres of
    // noise while sitting on the floor is normal.
    return last_state_.height_cm > 10.0f;
}

bool TelloDriver::enableVideoStream(bool enable) {
    return sendCommandAndWaitOk(enable ? "streamon" : "streamoff", config_.command_timeout_ms,
                                 config_.command_retries);
}

bool TelloDriver::emergencyStop() {
    std::lock_guard lock(command_mutex_);
    last_command_sent_at_.store(std::chrono::steady_clock::now(), std::memory_order_relaxed);
    return impl_->command_socket.sendTo(config_.ip, config_.command_port, "emergency");
}

void TelloDriver::sendVelocity(const common::VelocityCommand& velocity) {
    const auto clamp = [](int v) { return std::clamp(v, -100, 100); };
    const std::string cmd = "rc " + std::to_string(clamp(velocity.roll)) + " " +
                             std::to_string(clamp(velocity.pitch)) + " " +
                             std::to_string(clamp(velocity.throttle)) + " " +
                             std::to_string(clamp(velocity.yaw));

    // No lock here on purpose (see header comment): `rc` never reads a
    // reply, so it must never queue up behind a slow blocking receive() in
    // sendCommandAndWaitOk()/keepAliveLoop() - that shared-lock contention
    // was causing the periodic multi-hundred-ms control stalls.
    impl_->command_socket.sendTo(config_.ip, config_.command_port, cmd);
    last_command_sent_at_.store(std::chrono::steady_clock::now(), std::memory_order_relaxed);
}

DroneCapabilities TelloDriver::capabilities() const { return {}; }

void TelloDriver::stateReceiveLoop(std::stop_token token) {
    while (!token.stop_requested()) {
        auto line = impl_->state_socket.receive(/*timeoutMs=*/500);
        if (!line) continue;

        common::DroneState state;
        if (detail::parseTelloState(*line, state)) {
            state.received_at = std::chrono::steady_clock::now();
            {
                std::lock_guard lock(state_mutex_);
                last_state_ = state;
                have_state_ = true;
            }
            bus_.publish(state);
        }
    }
}

void TelloDriver::keepAliveLoop(std::stop_token token) {
    // Poll on a short period so a burst of real commands (e.g. takeoff, then
    // continuous `rc` streaming) is noticed promptly, but only actually send
    // the keep-alive ping once genuinely idle for keep_alive_interval_ms.
    constexpr auto kPollInterval = std::chrono::milliseconds(500);
    while (!token.stop_requested()) {
        std::this_thread::sleep_for(kPollInterval);
        if (token.stop_requested()) break;

        const auto idleFor = std::chrono::steady_clock::now() -
                              last_command_sent_at_.load(std::memory_order_relaxed);
        if (idleFor < std::chrono::milliseconds(config_.keep_alive_interval_ms)) continue;

        std::lock_guard lock(command_mutex_);
        impl_->command_socket.sendTo(config_.ip, config_.command_port, "command");
        last_command_sent_at_.store(std::chrono::steady_clock::now(), std::memory_order_relaxed);
        // Drain the ack so it doesn't sit in the socket buffer and get
        // mistaken for the response to a later, unrelated command.
        impl_->command_socket.receive(/*timeoutMs=*/500);
    }
}

} // namespace drivers::tello
