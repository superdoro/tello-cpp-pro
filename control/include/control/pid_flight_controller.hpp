#pragma once

#include <algorithm>

#include "control/frame_alignment.hpp"
#include "control/i_flight_controller.hpp"

namespace control {

struct PidGains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    // Cap on the accumulated integral term, in the same units as the error.
    float integral_limit = 1.0f;
};

struct FlightControllerConfig {
    // Position gains map metres of error to stick units. Forward/lateral are
    // in the body frame, so "forward" is always the drone's nose.
    PidGains forward{25.0f, 1.0f, 8.0f, 1.0f};
    PidGains lateral{25.0f, 1.0f, 8.0f, 1.0f};
    PidGains vertical{35.0f, 2.0f, 8.0f, 1.0f};
    // Yaw gains map RADIANS of error to stick units, which makes them easy to
    // under-set: at kp 40 a 15-degree misalignment is 0.27rad and asks for
    // only 11 of 100 sticks, so the airframe never squares up to the route.
    // Measured over a real flight that left a 15.4-degree average heading
    // error while saturating yaw just 6% of the time - soft, not starved.
    //
    // The integral term matters here more than on the other axes: a standing
    // heading offset produces a constant small error that proportional action
    // alone will never finish off.
    PidGains yaw{90.0f, 3.0f, 10.0f, 0.4f};

    // Forward stick held on open route, on top of what the PID asks for.
    //
    // Without this, cruise speed is an accident of the gains: pure pursuit
    // keeps the forward error pinned at the lookahead distance, so the
    // command settles at kp * lookahead and raising max_horizontal_command
    // changes nothing at all. Measured on a real flight, --max-speed 100 and
    // --max-speed 25 both flew at 0.7 m/s, because both were commanding
    // 25 * 1.55 ~= 39 sticks.
    //
    // Separating the two makes "how fast do I want to cruise" independent of
    // "how hard do I correct errors", which is what the gain actually exists
    // for. 0 keeps the original pure-PID behaviour.
    int cruise_command = 0;

    // The cruise term fades out linearly as the target comes within this
    // distance ahead, so the route's final waypoint is still a real stop
    // rather than something the drone is pushed through.
    float cruise_fade_m = 1.0f;

    // How hard to pull back onto the route, on top of chasing the lookahead
    // point. Expressed as a weight on the measured perpendicular offset,
    // which is then corrected by the same lateral/vertical PIDs - so the
    // integral term gets to remove a standing offset rather than letting the
    // drone settle parallel to the path, a metre to one side of it.
    //
    // 0 restores plain pure pursuit.
    float path_centering = 1.0f;

    // `rc` channels accept [-100, 100]. These caps are deliberately far below
    // that: this project's failure mode is a drone accelerating away on a bad
    // pose, and a low ceiling turns that from a crash into a nuisance. Raise
    // only after the loop is proven.
    int max_horizontal_command = 25;
    int max_vertical_command = 30;
    int max_yaw_command = 60;

    // Errors within this of zero produce no command, so the drone settles
    // instead of hunting around the setpoint on SLAM pose noise.
    //
    // Applied as a SOFT deadband - the threshold is subtracted from the
    // error rather than the error being snapped to zero. A hard deadband
    // makes the error jump discontinuously as it crosses the boundary, and
    // the derivative term turns that step into a violent one-frame stick
    // spike (a 5cm step in 33ms reads as 1.5 m/s of closing speed).
    float position_deadband_m = 0.05f;
    float heading_deadband_rad = 0.05f;

    // Low-pass time constant for the derivative term, in seconds. SLAM
    // positions are noisy at the centimetre level and differentiating them
    // raw amplifies exactly that noise into the sticks.
    float derivative_filter_s = 0.12f;

    // A target that moves further than this in one step is treated as a new
    // setpoint, and the PID history is cleared to stop the derivative term
    // turning the jump into a stick slam.
    //
    // It cannot be near-zero: in continuous path-following the target is a
    // lookahead point that moves smoothly every single frame, and resetting
    // on that would leave nothing but proportional control.
    float target_jump_reset_m = 0.75f;
    // Same idea for the heading: ~30 degrees of setpoint step in one frame is
    // a new target, not a turn in progress.
    float target_jump_reset_rad = 0.5f;

    FrameAlignment alignment;
};

// Four independent PID loops - forward, lateral, vertical, yaw - closed on
// SLAM pose feedback.
//
// Independence is a real simplification, not an oversight: the Tello's own
// firmware already stabilizes attitude and holds altitude, so this layer is
// commanding velocities in a frame that is close enough to decoupled for a
// waypoint follower. It is not adequate for aggressive flight.
class PidFlightController : public IFlightController {
public:
    explicit PidFlightController(FlightControllerConfig config = {});

    common::VelocityCommand computeCommand(const common::PoseEstimate& current,
                                            const Waypoint& target,
                                            std::chrono::duration<double> dt) override;
    void reset() override;

    const FlightControllerConfig& config() const { return config_; }

    // Scales the cruise feedforward for the frame, in [0, 1]. Set from
    // MissionPlanner::cruiseScale() so the drone brakes for bends. Only the
    // feedforward is scaled; error correction keeps its full authority.
    void setCruiseScale(float scale) { cruise_scale_ = std::clamp(scale, 0.0f, 1.0f); }

    // Vector from the drone to the nearest point on the route, in MAP
    // coordinates, as reported by MissionPlanner::pathOffset().
    void setPathOffset(const common::Vector3& mapOffset) { path_offset_ = mapOffset; }

    // Position error in the body frame (metres) from the last computeCommand,
    // for logging and for MissionPlanner's arrival test.
    common::Vector3 lastBodyError() const { return last_body_error_; }
    float lastHeadingError() const { return last_heading_error_; }

private:
    struct PidState {
        float integral = 0.0f;
        float previous_error = 0.0f;
        float filtered_derivative = 0.0f;
        bool has_previous = false;
    };

    float step(PidState& state, const PidGains& gains, float error, double dt);

    // Zeroes derivative history when the setpoint moves. Without it, every
    // waypoint transition steps the error by the distance between waypoints
    // and the D term slams the sticks to their limits for one frame.
    void resetOnTargetChange(const Waypoint& target);

    FlightControllerConfig config_;
    PidState forward_{};
    PidState lateral_{};
    PidState vertical_{};
    PidState yaw_{};
    common::Vector3 last_body_error_{};
    float last_heading_error_ = 0.0f;
    float cruise_scale_ = 1.0f;
    common::Vector3 path_offset_{};
    common::Vector3 last_target_{};
    float last_target_heading_ = 0.0f;
    bool has_target_ = false;
};

} // namespace control
