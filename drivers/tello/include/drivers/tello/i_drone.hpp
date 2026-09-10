#pragma once

#include "common/types.hpp"

namespace drivers::tello {

struct DroneCapabilities {
    bool has_video = true;
    int max_speed_cm_s = 800;
};

// Abstraction over a controllable drone. TelloDriver is the real
// implementation; a MockDrone implementation (tests/mocks) lets control-loop
// and mission logic be exercised without hardware. apps/ and control/ should
// depend only on this interface, never on TelloDriver directly.
class IDrone {
public:
    virtual ~IDrone() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;

    virtual bool takeoff() = 0;
    virtual bool land() = 0;

    // Continuous velocity streaming (maps to Tello's `rc a b c d`). Expected
    // to be called repeatedly (e.g. every 20-100ms) while flying.
    virtual void sendVelocity(const common::VelocityCommand& velocity) = 0;

    // Immediately cuts power to the motors. Not a landing - reserved for
    // genuine emergencies.
    virtual bool emergencyStop() = 0;

    virtual DroneCapabilities capabilities() const = 0;
};

} // namespace drivers::tello
