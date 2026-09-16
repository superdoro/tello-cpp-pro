#pragma once

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

#include "drivers/tello/i_drone.hpp"

namespace mocks {

// An IDrone that flies nowhere and remembers everything.
//
// The interface comment in i_drone.hpp has promised this file since the
// project started. It exists so mission and control logic can be exercised
// against the exact command stream a real flight would produce, with no
// hardware, no UDP, and no risk.
class MockDrone : public drivers::tello::IDrone {
public:
    bool connect() override {
        ++connect_calls;
        connected = connect_succeeds;
        return connect_succeeds;
    }

    void disconnect() override {
        ++disconnect_calls;
        connected = false;
    }

    bool takeoff() override {
        ++takeoff_calls;
        if (!takeoff_succeeds) return false;
        airborne = true;
        return true;
    }

    bool land() override {
        ++land_calls;
        if (!land_succeeds) return false;
        airborne = false;
        return true;
    }

    bool enableVideoStream(bool enable) override {
        ++stream_calls;
        streaming = enable;
        return stream_succeeds;
    }

    void sendVelocity(const common::VelocityCommand& velocity) override {
        commands.push_back(velocity);
        last_command = velocity;
    }

    bool emergencyStop() override {
        ++emergency_calls;
        airborne = false;
        return true;
    }

    drivers::tello::DroneCapabilities capabilities() const override { return capabilities_; }

    // --- assertions helpers ----------------------------------------------

    // True if every command sent was a full stop. What "the drone did not
    // move" looks like from the outside.
    bool onlyEverHovered() const {
        for (const auto& c : commands) {
            if (c.roll != 0 || c.pitch != 0 || c.throttle != 0 || c.yaw != 0) return false;
        }
        return true;
    }

    int maxAbsPitch() const {
        int worst = 0;
        for (const auto& c : commands) worst = std::max(worst, std::abs(c.pitch));
        return worst;
    }

    void reset() { *this = MockDrone{}; }

    // --- scripted behaviour ----------------------------------------------
    bool connect_succeeds = true;
    bool takeoff_succeeds = true;
    bool land_succeeds = true;
    bool stream_succeeds = true;
    drivers::tello::DroneCapabilities capabilities_{};

    // --- recorded state ---------------------------------------------------
    bool connected = false;
    bool airborne = false;
    bool streaming = false;

    int connect_calls = 0;
    int disconnect_calls = 0;
    int takeoff_calls = 0;
    int land_calls = 0;
    int stream_calls = 0;
    int emergency_calls = 0;

    std::vector<common::VelocityCommand> commands;
    common::VelocityCommand last_command{};
};

} // namespace mocks
