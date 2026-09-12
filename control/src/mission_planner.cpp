#include "control/mission_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/logging.hpp"

namespace control {

void MissionPlanner::loadMission(Mission mission) {
    mission_ = std::move(mission);
    current_target_.reset();
    status_ = MissionStatus::Idle;
    index_ = 0;
    abort_reason_.clear();
    inside_tolerance_ = false;
    tracking_lost_ = false;
}

void MissionPlanner::start() {
    current_target_.reset();
    if (mission_.waypoints.empty()) {
        abort("mission has no waypoints");
        return;
    }
    status_ = MissionStatus::Running;
    index_ = 0;
    inside_tolerance_ = false;
    tracking_lost_ = false;
    speed_mps_ = 0.0f;
    cruise_scale_ = 1.0f;
    have_last_pose_ = false;
    waypoint_started_at_ = std::chrono::steady_clock::now();
    common::logInfo("MissionPlanner",
                     "mission '" + mission_.name + "' started, " +
                         std::to_string(mission_.waypoints.size()) + " waypoints");
}

void MissionPlanner::resume() {
    if (mission_.waypoints.empty() || index_ >= mission_.waypoints.size()) {
        abort("nothing left to resume");
        return;
    }
    status_ = MissionStatus::Running;
    inside_tolerance_ = false;
    tracking_lost_ = false;
    // The time spent under manual control must not count against the
    // waypoint's timeout.
    waypoint_started_at_ = std::chrono::steady_clock::now();
    common::logInfo("MissionPlanner",
                     "resumed at waypoint " + std::to_string(index_ + 1) + "/" +
                         std::to_string(mission_.waypoints.size()));
}

void MissionPlanner::abort(const std::string& reason) {
    if (status_ == MissionStatus::Aborted) return;
    status_ = MissionStatus::Aborted;
    abort_reason_ = reason;
    common::logError("MissionPlanner", "ABORT: " + reason);
}

std::optional<Waypoint> MissionPlanner::currentTarget() const {
    if (status_ != MissionStatus::Running && status_ != MissionStatus::Holding) return std::nullopt;
    if (index_ >= mission_.waypoints.size()) return std::nullopt;
    if (current_target_.has_value()) return current_target_;
    return mission_.waypoints[index_];
}

namespace {

common::Vector3 direction(const common::Vector3& from, const common::Vector3& to) {
    const common::Vector3 delta{to.x - from.x, to.y - from.y, to.z - from.z};
    const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    if (length < 1e-5f) return {0.0f, 0.0f, 0.0f};
    return {delta.x / length, delta.y / length, delta.z / length};
}

float angleBetween(const common::Vector3& a, const common::Vector3& b) {
    const float dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
    return std::acos(dot);
}

bool isZero(const common::Vector3& v) {
    return std::abs(v.x) < 1e-6f && std::abs(v.y) < 1e-6f && std::abs(v.z) < 1e-6f;
}

}  // namespace

float MissionPlanner::effectiveLookahead() const {
    const auto& config = mission_.config;
    const float scaled = config.lookahead_m + speed_mps_ * config.lookahead_time_s;
    return std::clamp(scaled, config.lookahead_m, std::max(config.lookahead_max_m,
                                                            config.lookahead_m));
}

void MissionPlanner::updateSpeed(const common::Vector3& position,
                                  std::chrono::steady_clock::time_point now) {
    if (!have_last_pose_) {
        have_last_pose_ = true;
        last_position_ = position;
        last_pose_at_ = now;
        return;
    }
    const float dt = std::chrono::duration<float>(now - last_pose_at_).count();
    last_pose_at_ = now;
    if (dt < 1e-3f || dt > 1.0f) {  // a gap this long means tracking was lost
        last_position_ = position;
        return;
    }
    const float instant = distance(position, last_position_) / dt;
    last_position_ = position;

    // Heavy smoothing on purpose: this only needs to track the trend over a
    // second or so, and a lookahead that jitters would steer the drone.
    constexpr float kSmoothing = 0.1f;
    speed_mps_ += kSmoothing * (instant - speed_mps_);
}

void MissionPlanner::updatePathOffset(const common::Vector3& position) {
    const auto& waypoints = mission_.waypoints;
    path_offset_ = {0.0f, 0.0f, 0.0f};
    if (waypoints.size() < 2) return;

    // Only the segments around the current index are considered. Searching
    // the whole route would let a later pass of a loop capture the drone and
    // pull it sideways onto a part of the path it has not reached yet.
    const std::size_t first = index_ > 1 ? index_ - 2 : 0;
    const std::size_t last = std::min(index_ + 2, waypoints.size() - 1);

    float bestDistance = std::numeric_limits<float>::max();
    for (std::size_t i = first; i < last; ++i) {
        const common::Vector3& a = waypoints[i].position;
        const common::Vector3& b = waypoints[i + 1].position;
        const common::Vector3 leg{b.x - a.x, b.y - a.y, b.z - a.z};
        const float legLengthSquared = leg.x * leg.x + leg.y * leg.y + leg.z * leg.z;
        if (legLengthSquared < 1e-8f) continue;

        const common::Vector3 fromA{position.x - a.x, position.y - a.y, position.z - a.z};
        const float t = std::clamp(
            (fromA.x * leg.x + fromA.y * leg.y + fromA.z * leg.z) / legLengthSquared, 0.0f, 1.0f);
        const common::Vector3 closest{a.x + leg.x * t, a.y + leg.y * t, a.z + leg.z * t};

        const float d = distance(position, closest);
        if (d < bestDistance) {
            bestDistance = d;
            path_offset_ = {closest.x - position.x, closest.y - position.y,
                             closest.z - position.z};
        }
    }
}

void MissionPlanner::updateCruiseScale(const common::Vector3& from) {
    const auto& config = mission_.config;
    const auto& waypoints = mission_.waypoints;

    cruise_scale_ = 1.0f;
    if (index_ + 1 >= waypoints.size()) return;

    // Braking distance grows with speed: at 2 m/s a bend 1.5s away is 3m
    // ahead, at walking pace it is well under a metre.
    const float braking =
        std::max(config.corner_brake_time_s * speed_mps_, config.lookahead_m);

    // Reference is the leg the drone is travelling ALONG, not the one it is
    // heading towards. Using the outgoing leg loses the bend the moment the
    // index lands on the corner waypoint - the turn reads as already behind
    // us - and the braking releases exactly when it is needed most. The
    // incoming leg also keeps the brake on through the turn, until the drone
    // is established on the new heading.
    const common::Vector3 reference =
        index_ > 0 ? direction(waypoints[index_ - 1].position, waypoints[index_].position)
                   : direction(waypoints[index_].position, waypoints[index_ + 1].position);
    if (isZero(reference)) return;

    float travelled = distance(from, waypoints[index_].position);
    for (std::size_t i = index_ + 1; i + 1 < waypoints.size(); ++i) {
        travelled += distance(waypoints[i - 1].position, waypoints[i].position);
        if (travelled > braking) break;

        const common::Vector3 leg =
            direction(waypoints[i].position, waypoints[i + 1].position);
        if (isZero(leg)) continue;

        // Severity is the turn away from the CURRENT heading, so a long
        // sweeping bend accumulates and slows the drone too.
        const float severity =
            std::clamp(angleBetween(reference, leg) / std::max(config.corner_full_slow_rad, 0.01f),
                        0.0f, 1.0f);
        const float proximity = 1.0f - std::clamp(travelled / braking, 0.0f, 1.0f);
        const float scale =
            1.0f - severity * proximity * (1.0f - config.corner_min_cruise_scale);
        cruise_scale_ = std::min(cruise_scale_, scale);
    }
}

Waypoint MissionPlanner::lookaheadTarget(const common::Vector3& from) const {
    const auto& waypoints = mission_.waypoints;
    float remaining = std::max(effectiveLookahead(), 0.01f);
    common::Vector3 previous = from;

    // The reference direction for "has the route bent?" is taken from the
    // PATH, not from the drone's position. Using drone->waypoint would make
    // it swing wildly as the drone passes close to a waypoint, so the corner
    // test would flicker on and off and jerk the target with it.
    const common::Vector3 referenceDirection =
        index_ + 1 < waypoints.size()
            ? direction(waypoints[index_].position, waypoints[index_ + 1].position)
            : common::Vector3{0.0f, 0.0f, 0.0f};

    // The HEADING has to be interpolated as well, not just the position.
    // Taking it straight from the waypoint being approached makes it jump by
    // the angle between consecutive waypoints every time the lookahead point
    // crosses one - and the yaw PID turns that step into a stick spike, even
    // though the position is moving perfectly smoothly.
    float previousHeading = index_ > 0 ? waypoints[index_ - 1].heading_rad
                                        : waypoints[index_].heading_rad;

    for (std::size_t i = index_; i < waypoints.size(); ++i) {
        const common::Vector3& next = waypoints[i].position;

        // Truncate at a bend rather than aiming through it. Only legs between
        // waypoints are considered - the drone's own offset from the route
        // is not a bend in the route.
        if (i > index_ && !isZero(referenceDirection)) {
            const common::Vector3 leg =
                direction(waypoints[i - 1].position, waypoints[i].position);
            if (!isZero(leg) && angleBetween(referenceDirection, leg) >
                                     mission_.config.lookahead_corner_limit_rad) {
                return waypoints[i - 1];
            }
        }

        const float segment = distance(previous, next);
        if (segment >= remaining && segment > 1e-4f) {
            const float t = remaining / segment;
            Waypoint target = waypoints[i];
            target.position = {previous.x + (next.x - previous.x) * t,
                                previous.y + (next.y - previous.y) * t,
                                previous.z + (next.z - previous.z) * t};
            // Shortest-arc blend, so turning past +/-180 degrees does not
            // sweep the long way round.
            target.heading_rad =
                previousHeading + wrapAngle(waypoints[i].heading_rad - previousHeading) * t;
            return target;
        }
        remaining -= segment;
        previous = next;
        previousHeading = waypoints[i].heading_rad;
    }
    // Ran off the end of the route: aim at its last point.
    return waypoints.back();
}

bool MissionPlanner::advancePassedWaypoints(const common::Vector3& position,
                                             std::chrono::steady_clock::time_point now) {
    const auto& waypoints = mission_.waypoints;
    const std::size_t before = index_;

    while (index_ + 1 < waypoints.size()) {
        const common::Vector3& here = waypoints[index_].position;
        const common::Vector3& next = waypoints[index_ + 1].position;

        const bool reached =
            distance(position, here) <= waypoints[index_].position_tolerance_m;

        // "Have we passed it?" is a question about direction, not distance.
        // Comparing "am I nearer the next one?" gets it wrong exactly where
        // it matters: just past a waypoint the drone is still nearer to the
        // one behind it, and the planner would keep flying backwards at it.
        // Projecting onto the leg answers it properly - positive means the
        // drone is beyond the plane through this waypoint, perpendicular to
        // the way out of it.
        const common::Vector3 leg{next.x - here.x, next.y - here.y, next.z - here.z};
        const common::Vector3 fromHere{position.x - here.x, position.y - here.y,
                                        position.z - here.z};
        const float projection =
            leg.x * fromHere.x + leg.y * fromHere.y + leg.z * fromHere.z;

        if (!reached && projection <= 0.0f) break;
        ++index_;
    }

    if (index_ != before) {
        // Restart the clock. Without this the per-waypoint timeout silently
        // becomes a whole-mission timeout: the drone flies past waypoint
        // after waypoint and is aborted mid-route for "not reaching" one it
        // passed seconds earlier.
        waypoint_started_at_ = now;
        common::logInfo("MissionPlanner",
                         "passed waypoint " + std::to_string(before + 1) + " -> now heading for " +
                             std::to_string(index_ + 1) + "/" + std::to_string(waypoints.size()));
        return true;
    }
    return false;
}

bool MissionPlanner::violatesGeofence(const common::Vector3& position) const {
    const auto& fence = mission_.config.geofence;
    const float radius =
        std::sqrt(position.x * position.x + position.y * position.y + position.z * position.z);
    if (radius > fence.max_radius_m) return true;

    // Altitude is measured along the map's up axis, which is generally not
    // any single coordinate axis - see FrameAlignment.
    const common::Vector3& up = alignment_.map_up;
    const float upLength = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
    if (upLength < 1e-6f) return false;
    const float altitude =
        (position.x * up.x + position.y * up.y + position.z * up.z) / upLength;
    return altitude > fence.max_altitude_m || altitude < fence.min_altitude_m;
}

void MissionPlanner::onPoseUpdate(const common::PoseEstimate& pose,
                                   std::chrono::steady_clock::time_point now) {
    if (status_ != MissionStatus::Running && status_ != MissionStatus::Holding) return;

    const bool trusted = pose.state == common::TrackingState::Ok &&
                          pose.confidence >= mission_.config.min_confidence;

    if (!trusted) {
        if (!tracking_lost_) {
            tracking_lost_ = true;
            tracking_lost_since_ = now;
            common::logWarn("MissionPlanner", "pose not trustworthy - holding position");
        }
        status_ = MissionStatus::Holding;
        inside_tolerance_ = false;
        if (now - tracking_lost_since_ > mission_.config.max_tracking_loss) {
            abort("tracking lost for longer than the configured limit");
        }
        return;
    }

    if (tracking_lost_) {
        tracking_lost_ = false;
        // The clock on the current waypoint restarts: the time spent blind
        // should not count against its timeout.
        waypoint_started_at_ = now;
        common::logInfo("MissionPlanner", "pose recovered - resuming");
    }
    status_ = MissionStatus::Running;

    updateSpeed(pose.pose.position, now);

    if (violatesGeofence(pose.pose.position)) {
        abort("geofence violated - pose is outside the permitted flight volume");
        return;
    }

    if (index_ >= mission_.waypoints.size()) {
        status_ = MissionStatus::Complete;
        return;
    }

    // Continuous mode: never stop at an intermediate waypoint. Advance past
    // whatever has been reached or overshot, and aim at a point further along
    // the path so the drone keeps flying instead of settling and restarting.
    if (mission_.config.continuous && !onFinalWaypoint()) {
        advancePassedWaypoints(pose.pose.position, now);
        if (onFinalWaypoint()) {
            current_target_ = mission_.waypoints[index_];
        } else {
            current_target_ = lookaheadTarget(pose.pose.position);
            updateCruiseScale(pose.pose.position);
            updatePathOffset(pose.pose.position);
            // The waypoint timeout still applies, so a drone stuck against
            // an obstacle does not grind forward forever.
            if (now - waypoint_started_at_ > mission_.waypoints[index_].timeout) {
                abort("no progress along the route for " +
                      std::to_string(mission_.waypoints[index_].timeout.count() / 1000) +
                      "s while heading for waypoint " + std::to_string(index_ + 1));
            }
            return;
        }
    } else {
        current_target_ = mission_.waypoints[index_];
    }

    const Waypoint& target = mission_.waypoints[index_];

    const float positionError = distance(pose.pose.position, target.position);
    bool withinTolerance = positionError <= target.position_tolerance_m;
    if (withinTolerance && target.hold_heading) {
        const float headingError =
            std::abs(wrapAngle(target.heading_rad - alignment_.headingInMap(pose.pose.orientation)));
        withinTolerance = headingError <= target.heading_tolerance_rad;
    }

    if (withinTolerance) {
        if (!inside_tolerance_) {
            inside_tolerance_ = true;
            inside_tolerance_since_ = now;
        } else if (now - inside_tolerance_since_ >= target.dwell) {
            common::logInfo("MissionPlanner",
                             "waypoint " + std::to_string(index_) + " reached" +
                                 (target.label.empty() ? "" : " (" + target.label + ")"));
            ++index_;
            inside_tolerance_ = false;
            waypoint_started_at_ = now;
            if (index_ >= mission_.waypoints.size()) {
                status_ = MissionStatus::Complete;
                common::logInfo("MissionPlanner", "mission complete");
            }
        }
        return;
    }

    inside_tolerance_ = false;
    if (now - waypoint_started_at_ > target.timeout) {
        abort("waypoint " + std::to_string(index_) + " not reached within its timeout");
    }
}

} // namespace control
