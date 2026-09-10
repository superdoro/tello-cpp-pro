#include "drivers/tello/tello_driver.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "common/logging.hpp"
#include "tello_state_parser.hpp"
#include "udp_socket.hpp"

namespace drivers::tello {

struct TelloDriver::Impl {
    detail::UdpSocket command_socket;
    detail::UdpSocket state_socket;
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
    if (!sendCommandAndWaitOk("command")) {
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

bool TelloDriver::sendCommandAndWaitOk(const std::string& command) {
    std::lock_guard lock(command_mutex_);
    for (int attempt = 0; attempt < config_.command_retries; ++attempt) {
        if (!impl_->command_socket.sendTo(config_.ip, config_.command_port, command)) {
            continue;
        }
        last_command_sent_at_.store(std::chrono::steady_clock::now(), std::memory_order_relaxed);
        if (auto response = impl_->command_socket.receive(config_.command_timeout_ms)) {
            common::logDebug("TelloDriver", "cmd '" + command + "' -> '" + *response + "'");
            if (response->find("ok") != std::string::npos) {
                return true;
            }
        }
        common::logWarn("TelloDriver", "no/failed ack for '" + command + "', retrying");
    }
    return false;
}

bool TelloDriver::takeoff() { return sendCommandAndWaitOk("takeoff"); }

bool TelloDriver::land() { return sendCommandAndWaitOk("land"); }

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
