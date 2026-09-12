#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>

#include "core/event_bus.hpp"
#include "drivers/tello/i_drone.hpp"

namespace drivers::tello {

struct TelloConnectionConfig {
    std::string ip = "192.168.10.1";
    std::uint16_t command_port = 8889;
    std::uint16_t state_port = 8890;
    // Tello replies to whatever local port we sent the command from, so the
    // command socket must bind to a fixed, known local port too.
    std::uint16_t local_command_port = 8889;
    // For quick commands that answer immediately (`command`, `streamon`).
    int command_timeout_ms = 3000;
    int command_retries = 3;

    // Takeoff and landing are physical manoeuvres: the drone does not answer
    // until it has finished, which takes several seconds. A 3s timeout made
    // the driver give up and retry mid-takeoff, and the drone - already
    // taking off - answered the retry with "error". The flight then looked
    // like a failed takeoff while the aircraft was in fact airborne.
    int takeoff_timeout_ms = 15000;
    int land_timeout_ms = 12000;
    // Only sent when no other command (including `rc` streaming) has gone out
    // for this long - kept comfortably under Tello's ~15s no-activity
    // auto-land timeout. Any outgoing command is assumed to reset that
    // timeout, so a flight loop that streams `rc` continuously should never
    // actually trigger this ping in practice.
    int keep_alive_interval_ms = 5000;
};

// Real IDrone implementation talking to a Tello EDU over its SDK 2.0 UDP
// protocol: a command/response channel (8889), a periodic state broadcast
// (8890), and (separately, in the video/ module) a raw H264 stream (11111).
class TelloDriver : public IDrone {
public:
    TelloDriver(TelloConnectionConfig config, core::EventBus& bus);
    ~TelloDriver() override;

    bool connect() override;
    void disconnect() override;
    bool takeoff() override;
    bool land() override;
    bool enableVideoStream(bool enable) override;
    void sendVelocity(const common::VelocityCommand& velocity) override;
    bool emergencyStop() override;
    DroneCapabilities capabilities() const override;

private:
    bool sendCommandAndWaitOk(const std::string& command, int timeoutMs, int attempts);

    // What the drone's own telemetry says about being off the ground, or
    // nullopt when no state packet has ever arrived. The three-way answer
    // matters: "definitely on the ground" lets land() stop trying, while
    // "unknown" must not - giving up on landing because the telemetry link
    // is down is how a drone gets abandoned in mid-air.
    std::optional<bool> airborneByTelemetry() const;
    void stateReceiveLoop(std::stop_token token);
    void keepAliveLoop(std::stop_token token);

    TelloConnectionConfig config_;
    core::EventBus& bus_;

    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Latest parsed state, cached so command handling can consult telemetry
    // rather than trusting an ack that may never arrive.
    mutable std::mutex state_mutex_;
    common::DroneState last_state_{};
    bool have_state_ = false;

    std::jthread state_thread_;
    std::jthread keep_alive_thread_;
    bool connected_ = false;
    // Serializes the discrete ack-waiting commands (takeoff/land/emergency)
    // and the keep-alive ping against each other, so a reply is never read by
    // the wrong caller. `sendVelocity()` deliberately does NOT take this lock:
    // it never reads a reply, and UDP sendto() from multiple threads on one
    // socket is already safe, so serializing it here would only add latency
    // by queuing high-rate `rc` sends behind slow blocking receive() calls.
    std::mutex command_mutex_;
    std::atomic<std::chrono::steady_clock::time_point> last_command_sent_at_{
        std::chrono::steady_clock::time_point{}};
};

} // namespace drivers::tello
