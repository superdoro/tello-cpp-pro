#pragma once

#include <chrono>

#include "common/types.hpp"
#include "control/waypoint.hpp"

namespace control {

// Turns "where I am" + "where I want to be" into stick commands. Kept behind
// an interface so the PID implementation can be replaced (with an MPC, say)
// without touching the mission logic.
class IFlightController {
public:
    virtual ~IFlightController() = default;

    virtual common::VelocityCommand computeCommand(const common::PoseEstimate& current,
                                                    const Waypoint& target,
                                                    std::chrono::duration<double> dt) = 0;

    // Clears integral terms and derivative history. MUST be called whenever
    // the pose feed is interrupted (tracking lost) or the target changes:
    // otherwise the integrator winds up during the blind period and the drone
    // lurches when tracking returns.
    virtual void reset() = 0;
};

} // namespace control
