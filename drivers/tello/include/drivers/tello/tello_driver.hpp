#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
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
    int command_timeout_ms = 3000;
    int command_retries = 3;
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
    void sendVelocity(const common::VelocityCommand& velocity) override;
    bool emergencyStop() override;
    DroneCapabilities capabilities() const override;

private:
    bool sendCommandAndWaitOk(const std::string& command);
    void stateReceiveLoop(std::stop_token token);
    void keepAliveLoop(std::stop_token token);

    TelloConnectionConfig config_;
    core::EventBus& bus_;

    struct Impl;
    std::unique_ptr<Impl> impl_;

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
