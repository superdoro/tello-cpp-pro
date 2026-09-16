#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>

#include "common/frame_alignment.hpp"
#include "common/obstacle.hpp"
#include "common/types.hpp"
#include "control/waypoint.hpp"

namespace control {

enum class AvoidanceState : std::uint8_t {
    Disabled,  // off, not built, or the pose is not trustworthy anyway
    Degraded,  // running, but the picture is too old//weak to act on beyond a speed cap
    Clear,     // nothing within slow_distance
    Slowing,   // easing off for something ahead
    Steering,  // also aiming to one side of it
    Braking,   // holding position: it is too close to fly past
    Blocked,   // braking, and it has not improved for blocked_abort_after
};

const char* avoidanceStateName(AvoidanceState state);

// What the avoidance layer wants done to this control step.
//
// Every field is shaped so that "do nothing" is representable and is the
// default: an identity decision leaves the flight exactly as it would have
// been without this feature, which is the property the whole design rests on.
struct AvoidanceDecision {
    AvoidanceState state = AvoidanceState::Disabled;

    // MULTIPLIER in [0, 1], applied on top of MissionPlanner::cruiseScale().
    // The type is the guarantee: a product of two values in [0, 1] cannot
    // exceed either, so this can only ever slow the drone down.
    float cruise_scale = 1.0f;

    // Added to the planner's target position, in MAP coordinates.
    common::Vector3 target_offset_map{};

    // Scales MissionPlanner::pathOffset(). The path-centring term actively
    // pulls the drone back onto the route and would fight a lateral escape,
    // so it is eased off while steering - and its natural pull is then what
    // rejoins the route for free once the offset decays.
    float path_offset_scale = 1.0f;

    // Hover this step. The app zeroes the command and resets the PID, which
    // is exactly what the existing "pose untrustworthy" branch already does.
    bool hold = false;

    // Diagnostics.
    float clearance_m = std::numeric_limits<float>::infinity();
    float snapshot_age_s = 0.0f;
    common::Vector3 offset_body{};  // the same offset before rotation into the map
    const char* reason = "";
};

// Turns an obstacle picture into a modification of one control step.
//
// It sits BESIDE MissionPlanner and PidFlightController rather than inside
// either: the planner owns the route and every reason to stop flying, the
// controller owns the sticks, and this owns "given what is in front of us,
// how should this step differ". It never talks to the drone and never aborts
// a mission itself - blockedTooLong() reports, and the app turns that into
// MissionPlanner::abort(), preserving the planner's ownership of abort
// reasons.
//
// Depends only on common:: types, so all of it is exercisable with synthetic
// snapshots and no perception stack at all.
class ObstacleAvoidance {
public:
    explicit ObstacleAvoidance(AvoidanceConfig config = {});

    void setConfig(const AvoidanceConfig& config);
    const AvoidanceConfig& config() const { return config_; }

    // Must match the flight controller's, or an escape computed here will be
    // flown in a different direction than it was chosen in.
    void setAlignment(const common::FrameAlignment& alignment) { alignment_ = alignment; }

    // The geofence an offset must not push the target outside of. Copied from
    // the mission by the app; an offset that violated it would make the
    // planner abort on the very next pose.
    void setGeofence(const GeofenceConfig& geofence) { geofence_ = geofence; }

    // Current ground speed, from MissionPlanner::speed(), used to charge the
    // snapshot's age against the clearance. Defaults to 0, which makes that
    // compensation a no-op - the safe direction for a value nobody set.
    void setSpeed(float metresPerSecond) { speed_mps_ = metresPerSecond; }

    // The whole interface. Returns an identity decision - cruise_scale 1,
    // zero offset, no hold - whenever it is off, the pose is untrustworthy,
    // or there is nothing to act on.
    AvoidanceDecision evaluate(const common::PoseEstimate& pose, const Waypoint& target,
                                const std::optional<common::ObstacleSnapshot>& snapshot,
                                std::chrono::steady_clock::time_point now);

    AvoidanceState state() const { return last_.state; }
    const AvoidanceDecision& lastDecision() const { return last_; }

    // Held at a standstill long enough that something else has to happen.
    bool blockedTooLong() const { return blocked_; }
    // Only meaningful with require_perception set.
    bool unhealthyTooLong() const { return unhealthy_too_long_; }

    void reset();

private:
    // Nearest occupied range inside the forward corridor, and whether the
    // corridor was observed at all. Computed here rather than taken from the
    // snapshot so that the corridor is control's policy, sized to the
    // airframe, not perception's.
    struct Corridor {
        float clearance_m = std::numeric_limits<float>::infinity();
        bool observed = false;
    };
    Corridor measureCorridor(const common::ObstacleSnapshot& snapshot) const;

    // Best direction to aim aside towards, as a unit vector in the BODY
    // frame, or nullopt when nothing qualifies.
    std::optional<common::Vector3> chooseEscape(const common::ObstacleSnapshot& snapshot,
                                                 const common::Vector3& targetBody,
                                                 std::chrono::steady_clock::time_point now);

    // Moves the held offset towards `desired` at no more than the configured
    // rate, and returns the new value. Both are in the BODY frame.
    common::Vector3 slewOffset(const common::Vector3& desired,
                                std::chrono::steady_clock::time_point now);

    bool offsetBreaksGeofence(const common::Vector3& position,
                               const common::Vector3& offsetMap) const;

    // Applies the dwell time and the Schmitt exit margin to a proposed state.
    AvoidanceState settleState(AvoidanceState proposed,
                                std::chrono::steady_clock::time_point now);

    AvoidanceDecision identity(AvoidanceState state, const char* reason,
                                std::chrono::steady_clock::time_point now);

    AvoidanceConfig config_;
    common::FrameAlignment alignment_;
    GeofenceConfig geofence_;
    float speed_mps_ = 0.0f;

    AvoidanceDecision last_;

    // Held across steps so a manoeuvre is continuous rather than re-decided
    // from scratch every frame.
    common::Vector3 offset_body_{};
    common::Vector3 committed_escape_{};
    bool have_commitment_ = false;
    std::chrono::steady_clock::time_point committed_at_{};

    AvoidanceState state_ = AvoidanceState::Disabled;
    std::chrono::steady_clock::time_point state_since_{};
    std::chrono::steady_clock::time_point last_step_at_{};
    bool have_last_step_ = false;

    std::chrono::steady_clock::time_point braking_since_{};
    float best_clearance_while_braking_ = std::numeric_limits<float>::infinity();
    bool blocked_ = false;

    std::chrono::steady_clock::time_point first_seen_at_{};
    bool have_first_seen_ = false;
    std::chrono::steady_clock::time_point unhealthy_since_{};
    bool unhealthy_ = false;
    bool unhealthy_too_long_ = false;
};

} // namespace control
