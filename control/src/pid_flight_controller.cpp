#include "control/pid_flight_controller.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

int toStick(float value, int limit) {
    const float clamped = std::clamp(value, -static_cast<float>(limit), static_cast<float>(limit));
    return static_cast<int>(std::lround(clamped));
}

// Shrinks the error towards zero by `width` instead of snapping it to zero,
// so the command is continuous across the deadband boundary.
float softDeadband(float error, float width) {
    if (error > width) return error - width;
    if (error < -width) return error + width;
    return 0.0f;
}

}  // namespace

PidFlightController::PidFlightController(FlightControllerConfig config)
    : config_(std::move(config)) {}

void PidFlightController::reset() {
    forward_ = {};
    lateral_ = {};
    vertical_ = {};
    yaw_ = {};
    last_body_error_ = {};
    last_heading_error_ = 0.0f;
    // Deliberately NOT clearing last_target_: reset() is called on tracking
    // loss too, and forgetting the target there would make the next frame
    // look like a waypoint change and reset all over again (harmless, but it
    // hides real transitions from resetOnTargetChange).
}

float PidFlightController::step(PidState& state, const PidGains& gains, float error, double dt) {
    if (state.has_previous && dt > 1e-4) {
        const float raw = static_cast<float>((error - state.previous_error) / dt);
        // First-order low pass. alpha = dt / (tau + dt), so a large tau
        // trusts history and a tau of zero passes the raw derivative.
        const float tau = std::max(config_.derivative_filter_s, 0.0f);
        const float alpha = tau > 1e-6f ? static_cast<float>(dt) / (tau + static_cast<float>(dt))
                                         : 1.0f;
        state.filtered_derivative += alpha * (raw - state.filtered_derivative);
    }
    state.previous_error = error;
    state.has_previous = true;

    if (dt > 1e-4) {
        state.integral += error * static_cast<float>(dt);
        state.integral = std::clamp(state.integral, -gains.integral_limit, gains.integral_limit);
    }

    return gains.kp * error + gains.ki * state.integral + gains.kd * state.filtered_derivative;
}

void PidFlightController::resetOnTargetChange(const Waypoint& target) {
    const bool moved =
        !has_target_ || distance(target.position, last_target_) > config_.target_jump_reset_m;
    const bool turned =
        !has_target_ || std::abs(wrapAngle(target.heading_rad - last_target_heading_)) >
                             config_.target_jump_reset_rad;
    last_target_ = target.position;
    last_target_heading_ = target.heading_rad;
    has_target_ = true;
    if (moved || turned) reset();
}

common::VelocityCommand PidFlightController::computeCommand(const common::PoseEstimate& current,
                                                             const Waypoint& target,
                                                             std::chrono::duration<double> dt) {
    // Refusing to act on an untracked pose belongs here as well as in the
    // mission planner: a controller that can only ever be wrong should
    // command a hover, whatever its caller believes.
    if (current.state != common::TrackingState::Ok) {
        reset();
        return {};
    }

    resetOnTargetChange(target);

    const common::Vector3 mapError{target.position.x - current.pose.position.x,
                                    target.position.y - current.pose.position.y,
                                    target.position.z - current.pose.position.z};

    common::Vector3 bodyError = config_.alignment.mapToBody(current.pose.orientation, mapError);

    // Add the pull back towards the route itself. Only sideways and
    // vertically: nudging the along-path component would fight the cruise
    // term for control of speed.
    if (config_.path_centering != 0.0f) {
        const common::Vector3 bodyOffset =
            config_.alignment.mapToBody(current.pose.orientation, path_offset_);
        bodyError.y += config_.path_centering * bodyOffset.y;
        bodyError.z += config_.path_centering * bodyOffset.z;
    }
    last_body_error_ = bodyError;

    const auto deadbanded = [this](float error) {
        return softDeadband(error, config_.position_deadband_m);
    };

    const double seconds = dt.count();
    float forward = step(forward_, config_.forward, deadbanded(bodyError.x), seconds);

    if (config_.cruise_command > 0) {
        // Only ever pushes towards a target that is genuinely ahead: a
        // negative forward error means the target is behind, and cruising
        // into it would drive the drone further away.
        const float fade =
            std::clamp(bodyError.x / std::max(config_.cruise_fade_m, 0.1f), 0.0f, 1.0f);
        forward += static_cast<float>(config_.cruise_command) * fade * cruise_scale_;
    }
    const float lateral = step(lateral_, config_.lateral, deadbanded(bodyError.y), seconds);
    const float vertical = step(vertical_, config_.vertical, deadbanded(bodyError.z), seconds);

    float headingError = 0.0f;
    if (target.hold_heading) {
        headingError = softDeadband(
            wrapAngle(target.heading_rad - config_.alignment.headingInMap(current.pose.orientation)),
            config_.heading_deadband_rad);
    }
    last_heading_error_ = headingError;
    const float yaw = step(yaw_, config_.yaw, headingError, seconds);

    common::VelocityCommand command;
    command.pitch = toStick(forward, config_.max_horizontal_command);
    // `rc` roll is positive to the RIGHT; the body frame's y is positive to
    // the LEFT. This sign flip is the single most error-prone line in the
    // control path.
    command.roll = toStick(-lateral, config_.max_horizontal_command);
    command.throttle = toStick(vertical, config_.max_vertical_command);
    // `rc` yaw is positive CLOCKWISE seen from above; headings here are
    // positive counter-clockwise. Same story.
    command.yaw = toStick(-yaw, config_.max_yaw_command);
    return command;
}

} // namespace control
