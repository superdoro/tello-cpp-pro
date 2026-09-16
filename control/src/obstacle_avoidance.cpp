#include "control/obstacle_avoidance.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

using common::ObstacleSnapshot;

constexpr float kPi = ObstacleSnapshot::kPi;

// How far a bin's own angular footprint reaches beyond its centre direction.
// Bins are 15 degrees of azimuth by 18 of elevation, so treating only the
// centre direction as occupied would let an obstacle sitting at a bin's edge
// read as outside the corridor. Half the bin's diagonal, applied as a widening
// of everything, keeps the error on the conservative side.
const float kBinHalfSpreadRad =
    0.5f * std::sqrt(ObstacleSnapshot::kAzimuthBinRad * ObstacleSnapshot::kAzimuthBinRad +
                      ObstacleSnapshot::kElevationBinRad * ObstacleSnapshot::kElevationBinRad);

// Escape candidates are limited to the forward hemisphere, well inside it.
// Nothing further out is reachable without a yaw, and this camera cannot see
// past about 27 degrees anyway.
constexpr float kEscapeMaxAzimuthRad = 60.0f * kPi / 180.0f;
constexpr float kEscapeMaxElevationRad = 30.0f * kPi / 180.0f;

// Scoring weights, all in metres per radian so they trade off directly against
// the clearance term.
constexpr float kWeightTowardsTarget = 1.5f;
constexpr float kWeightCommitment = 2.0f;

// Staying level is weighted well above simply pointing at the target.
//
// At 1.0 this weight made "descend 18 degrees" score 3.2146 and "sidestep 30
// degrees" score 3.2146 - an exact tie, because the bins are 15 degrees of
// azimuth and 18 of elevation and 1.5 x 30 equals 2.5 x 18. The tie fell to
// whichever the loop reached first, which was the descent, so the drone would
// have dived at the floor to avoid a wall. Vertical escapes stay available
// when nothing lateral qualifies; they are just no longer free.
constexpr float kWeightLevelFlight = 2.5f;

// And going down is worse than going up by the same angle. At normal indoor
// flying height the floor is nearer than the ceiling, and descending puts the
// aircraft into its own ground effect.
constexpr float kDescentPenalty = 0.3f;

// A clearance must improve by this much to count as progress rather than
// noise, for the purpose of the blocked timer.
constexpr float kProgressEpsilonM = 0.15f;

constexpr std::chrono::milliseconds kUnhealthyAbortAfter{2000};

// Longest step the slew limiter will integrate over. A pause (a breakpoint, a
// stalled feed) must not become licence for one huge jump.
constexpr float kMaxSlewDtS = 0.5f;

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

float smoothstep(float edge0, float edge1, float x) {
    if (edge1 <= edge0) return x >= edge1 ? 1.0f : 0.0f;
    const float t = clamp01((x - edge0) / (edge1 - edge0));
    return t * t * (3.0f - 2.0f * t);
}

float lengthOf(const common::Vector3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float dot(const common::Vector3& a, const common::Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

common::Vector3 scaled(const common::Vector3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }

common::Vector3 added(const common::Vector3& a, const common::Vector3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

common::Vector3 subtracted(const common::Vector3& a, const common::Vector3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

common::Vector3 unit(const common::Vector3& v) {
    const float l = lengthOf(v);
    return l > 1e-6f ? scaled(v, 1.0f / l) : common::Vector3{};
}

float angleBetweenUnit(const common::Vector3& a, const common::Vector3& b) {
    return std::acos(std::clamp(dot(a, b), -1.0f, 1.0f));
}

// Is a point at `range` along a unit direction inside the forward corridor?
//
// The corridor is a cone-ish tube: radius `radius0` at the drone, flaring by
// `growth` per metre travelled. For a direction at angle t from the nose, a
// point at distance r sits x = r*cos(t) along the axis and p = r*sin(t) off
// it, so it is inside when  r*(sin t - growth*cos t) <= radius0.
bool insideCorridor(const common::Vector3& direction, float range, float radius0, float growth) {
    if (direction.x <= 0.0f || !(range > 0.0f)) return false;
    const float angle = std::acos(std::clamp(direction.x, -1.0f, 1.0f));
    const float effective = std::max(0.0f, angle - kBinHalfSpreadRad);
    return range * (std::sin(effective) - growth * std::cos(effective)) <= radius0;
}

int severity(AvoidanceState state) {
    switch (state) {
        case AvoidanceState::Disabled: return -1;
        case AvoidanceState::Degraded: return 0;
        case AvoidanceState::Clear: return 0;
        case AvoidanceState::Slowing: return 1;
        case AvoidanceState::Steering: return 2;
        case AvoidanceState::Braking: return 3;
        case AvoidanceState::Blocked: return 4;
    }
    return 0;
}

}  // namespace

const char* avoidanceStateName(AvoidanceState state) {
    switch (state) {
        case AvoidanceState::Disabled: return "OFF";
        case AvoidanceState::Degraded: return "DEGRADED";
        case AvoidanceState::Clear: return "CLEAR";
        case AvoidanceState::Slowing: return "SLOWING";
        case AvoidanceState::Steering: return "STEERING";
        case AvoidanceState::Braking: return "BRAKING";
        case AvoidanceState::Blocked: return "BLOCKED";
    }
    return "?";
}

ObstacleAvoidance::ObstacleAvoidance(AvoidanceConfig config) : config_(config) {}

void ObstacleAvoidance::setConfig(const AvoidanceConfig& config) { config_ = config; }

void ObstacleAvoidance::reset() {
    last_ = AvoidanceDecision{};
    offset_body_ = {};
    committed_escape_ = {};
    have_commitment_ = false;
    state_ = AvoidanceState::Disabled;
    have_last_step_ = false;
    best_clearance_while_braking_ = std::numeric_limits<float>::infinity();
    blocked_ = false;
    have_first_seen_ = false;
    unhealthy_ = false;
    unhealthy_too_long_ = false;
}

AvoidanceDecision ObstacleAvoidance::identity(AvoidanceState state, const char* reason,
                                               std::chrono::steady_clock::time_point now) {
    AvoidanceDecision decision;
    decision.state = state;
    decision.reason = reason;
    offset_body_ = {};
    have_commitment_ = false;
    blocked_ = false;
    if (state_ != state) {
        state_ = state;
        state_since_ = now;
    }
    last_step_at_ = now;
    have_last_step_ = true;
    last_ = decision;
    return decision;
}

ObstacleAvoidance::Corridor ObstacleAvoidance::measureCorridor(
    const common::ObstacleSnapshot& snapshot) const {
    Corridor corridor;

    for (int el = 0; el < ObstacleSnapshot::kElevationBins; ++el) {
        for (int az = 0; az < ObstacleSnapshot::kAzimuthBins; ++az) {
            const common::ObstacleBin& bin = snapshot.at(az, el);
            if (!bin.observed()) continue;

            const common::Vector3 direction = ObstacleSnapshot::binDirection(az, el);
            if (direction.x <= 0.0f) continue;

            // "Is the corridor observed at all" is asked at the brake
            // distance: what matters is whether there is evidence about the
            // space the drone is about to occupy, not about the far field.
            if (insideCorridor(direction, config_.brake_distance_m, config_.corridor_radius_m,
                                config_.corridor_radius_growth)) {
                corridor.observed = true;
            }

            if (!bin.occupied() || !std::isfinite(bin.range_m)) continue;
            if (!insideCorridor(direction, bin.range_m, config_.corridor_radius_m,
                                 config_.corridor_radius_growth)) {
                continue;
            }
            corridor.clearance_m = std::min(corridor.clearance_m, bin.range_m);
        }
    }
    return corridor;
}

std::optional<common::Vector3> ObstacleAvoidance::chooseEscape(
    const common::ObstacleSnapshot& snapshot, const common::Vector3& targetBody,
    std::chrono::steady_clock::time_point now) {
    const bool committed =
        have_commitment_ && (now - committed_at_) < config_.commit_time;

    std::optional<common::Vector3> best;
    float bestScore = -std::numeric_limits<float>::infinity();

    for (int el = 0; el < ObstacleSnapshot::kElevationBins; ++el) {
        if (std::abs(ObstacleSnapshot::elevationBinCentre(el)) > kEscapeMaxElevationRad) continue;

        for (int az = 0; az < ObstacleSnapshot::kAzimuthBins; ++az) {
            const float azimuth = ObstacleSnapshot::azimuthBinCentre(az);
            if (std::abs(azimuth) > kEscapeMaxAzimuthRad) continue;

            const common::ObstacleBin& bin = snapshot.at(az, el);

            // Hard rejects, in order of how badly getting them wrong ends.
            //
            // Unobserved is the important one: with a 55-degree field of view
            // most of this snapshot is unknown at any instant, and an
            // unobserved bin reads as infinitely clear. Flying at one is
            // flying at whatever happens to be there.
            if (!bin.observed()) continue;
            if (bin.dynamic()) continue;
            if (bin.range_m < config_.steer_distance_m) continue;

            const common::Vector3 direction = ObstacleSnapshot::binDirection(az, el);
            if (direction.x <= 0.0f) continue;

            float score = std::min(bin.range_m, config_.slow_distance_m);
            if (lengthOf(targetBody) > 1e-3f) {
                score -= kWeightTowardsTarget * angleBetweenUnit(direction, unit(targetBody));
            }
            if (committed) {
                score -= kWeightCommitment * angleBetweenUnit(direction, committed_escape_);
            }
            // Prefer staying level. Climbing over or dropping under something
            // stays available when nothing sideways qualifies, but indoors the
            // ceiling and the floor are closer than the walls.
            const float elevation = ObstacleSnapshot::elevationBinCentre(el);
            score -= kWeightLevelFlight * std::abs(elevation);
            if (elevation < 0.0f) score -= kDescentPenalty;

            if (score > bestScore) {
                bestScore = score;
                best = direction;
            }
        }
    }
    return best;
}

common::Vector3 ObstacleAvoidance::slewOffset(const common::Vector3& desired,
                                               std::chrono::steady_clock::time_point now) {
    float dt = 0.0f;
    if (have_last_step_) {
        dt = std::chrono::duration<float>(now - last_step_at_).count();
        dt = std::clamp(dt, 0.0f, kMaxSlewDtS);
    }

    const common::Vector3 delta = subtracted(desired, offset_body_);
    const float distance = lengthOf(delta);
    const float maxStep = std::max(0.0f, config_.offset_rate_m_per_s) * dt;

    if (distance <= maxStep || distance < 1e-6f) {
        offset_body_ = desired;
    } else {
        offset_body_ = added(offset_body_, scaled(delta, maxStep / distance));
    }
    return offset_body_;
}

bool ObstacleAvoidance::offsetBreaksGeofence(const common::Vector3& position,
                                              const common::Vector3& offsetMap) const {
    const common::Vector3 moved = added(position, offsetMap);
    if (lengthOf(moved) > geofence_.max_radius_m) return true;

    const common::Vector3 up = unit(alignment_.map_up);
    if (lengthOf(up) < 1e-6f) return false;
    const float altitude = dot(moved, up);
    return altitude > geofence_.max_altitude_m || altitude < geofence_.min_altitude_m;
}

AvoidanceState ObstacleAvoidance::settleState(AvoidanceState proposed,
                                               std::chrono::steady_clock::time_point now) {
    if (proposed == state_) return state_;

    // Escalation is immediate; only relaxing waits out the dwell. A drone that
    // has to wait 400 ms to start braking has travelled another 40 cm.
    const bool escalating = severity(proposed) > severity(state_);
    if (!escalating && (now - state_since_) < config_.min_state_dwell) return state_;

    state_ = proposed;
    state_since_ = now;
    return state_;
}

AvoidanceDecision ObstacleAvoidance::evaluate(
    const common::PoseEstimate& pose, const Waypoint& target,
    const std::optional<common::ObstacleSnapshot>& snapshot,
    std::chrono::steady_clock::time_point now) {
    if (!have_first_seen_) {
        have_first_seen_ = true;
        first_seen_at_ = now;
    }

    // --- reasons to do nothing at all ------------------------------------
    //
    // Both of these return an EXACT identity decision. That is the property
    // the whole feature rests on: with avoidance off, or with a pose the
    // planner is already refusing to act on, the flight is bit-for-bit what it
    // would have been without any of this.
    if (!config_.enabled) {
        unhealthy_ = false;
        unhealthy_too_long_ = false;
        return identity(AvoidanceState::Disabled, "avoidance off", now);
    }
    if (pose.state != common::TrackingState::Ok) {
        // The planner's Holding path already owns this case, and it hovers.
        unhealthy_ = false;
        unhealthy_too_long_ = false;
        return identity(AvoidanceState::Disabled, "pose not trustworthy", now);
    }

    // --- how much of the picture can be believed --------------------------
    const char* degradedReason = nullptr;
    float ageSeconds = 0.0f;
    bool startingUp = false;

    if (!snapshot.has_value()) {
        startingUp = (now - first_seen_at_) < config_.startup_grace;
        degradedReason = startingUp ? "starting up" : "no obstacle data";
    } else {
        ageSeconds = std::chrono::duration<float>(now - snapshot->stamp).count();
        if (ageSeconds < 0.0f) ageSeconds = 0.0f;
        if (snapshot->health != common::PerceptionHealth::Ok) {
            degradedReason = "perception unhealthy";
        } else if (std::chrono::duration<float>(now - snapshot->stamp) > config_.max_snapshot_age) {
            degradedReason = "obstacle data stale";
        } else if (snapshot->depth_scale_confidence < config_.min_confidence) {
            degradedReason = "depth scale not confident";
        }
    }

    // A missing snapshot inside the startup grace is not yet a fault, so it
    // must not count towards require_perception's abort.
    const bool faulted = degradedReason != nullptr && !startingUp;
    if (faulted) {
        if (!unhealthy_) {
            unhealthy_ = true;
            unhealthy_since_ = now;
        }
        unhealthy_too_long_ =
            config_.require_perception && (now - unhealthy_since_) > kUnhealthyAbortAfter;
    } else {
        unhealthy_ = false;
        unhealthy_too_long_ = false;
    }

    AvoidanceDecision decision;
    decision.snapshot_age_s = ageSeconds;

    if (degradedReason != nullptr) {
        // Degraded may only slow the drone. It may not steer, so the held
        // offset decays to zero at the slew rate rather than snapping - a
        // snap would be a target jump big enough to spike the PID's
        // derivative term.
        blocked_ = false;
        have_commitment_ = false;
        decision.state = settleState(AvoidanceState::Degraded, now);
        decision.reason = degradedReason;
        // Waiting for the first snapshot is not a fault, and must not cost
        // speed - otherwise merely enabling the feature slows every takeoff.
        decision.cruise_scale = startingUp ? 1.0f : clamp01(config_.degraded_cruise_scale);
        decision.offset_body = slewOffset({}, now);
        decision.target_offset_map =
            alignment_.bodyToMap(pose.pose.orientation, decision.offset_body);
        decision.path_offset_scale = 1.0f;
        last_step_at_ = now;
        have_last_step_ = true;
        last_ = decision;
        return decision;
    }

    // --- act on it --------------------------------------------------------
    const Corridor corridor = measureCorridor(*snapshot);

    if (!corridor.observed) {
        // Nothing is known about the space ahead. That is not the same as it
        // being clear, and the honest response is the degraded speed cap.
        blocked_ = false;
        have_commitment_ = false;
        decision.state = settleState(AvoidanceState::Degraded, now);
        decision.reason = "corridor unobserved";
        decision.cruise_scale = clamp01(config_.degraded_cruise_scale);
        decision.offset_body = slewOffset({}, now);
        decision.target_offset_map =
            alignment_.bodyToMap(pose.pose.orientation, decision.offset_body);
        last_step_at_ = now;
        have_last_step_ = true;
        last_ = decision;
        return decision;
    }

    // Charge the snapshot's age against the clearance: in the time since that
    // frame was captured the drone has closed some of the gap.
    const float clearance =
        std::max(0.0f, corridor.clearance_m - std::max(0.0f, speed_mps_) * ageSeconds);
    decision.clearance_m = corridor.clearance_m;

    // Which band, with a Schmitt margin so a clearance hovering on a threshold
    // does not chatter between states.
    const bool inBrake =
        state_ == AvoidanceState::Braking || state_ == AvoidanceState::Blocked;
    const float brakeThreshold =
        config_.brake_distance_m + (inBrake ? config_.exit_margin_m : 0.0f);
    const float steerThreshold = config_.steer_distance_m +
                                  (state_ == AvoidanceState::Steering ? config_.exit_margin_m : 0.0f);
    const float slowThreshold = config_.slow_distance_m +
                                 (state_ == AvoidanceState::Slowing ? config_.exit_margin_m : 0.0f);

    AvoidanceState proposed = AvoidanceState::Clear;
    if (config_.enable_brake && clearance <= brakeThreshold) {
        proposed = AvoidanceState::Braking;
    } else if (config_.enable_steer && clearance <= steerThreshold) {
        proposed = AvoidanceState::Steering;
    } else if (config_.enable_slow && clearance <= slowThreshold) {
        proposed = AvoidanceState::Slowing;
    }

    // Steering additionally needs to trust the scale enough to pick a
    // direction, and needs somewhere to go.
    const bool maySteer = config_.enable_steer &&
                           snapshot->depth_scale_confidence >= config_.steer_min_confidence;

    common::Vector3 desiredOffset{};
    if (proposed == AvoidanceState::Steering || proposed == AvoidanceState::Braking) {
        if (maySteer) {
            const common::Vector3 targetBody = alignment_.mapToBody(
                pose.pose.orientation, subtracted(target.position, pose.pose.position));
            if (const auto escape = chooseEscape(*snapshot, targetBody, now)) {
                const float sideways = std::sqrt(escape->y * escape->y + escape->z * escape->z);
                if (sideways > 1e-3f) {
                    // Urgency runs from 0 at the steer distance to 1 at the
                    // brake distance, so the manoeuvre grows as the thing gets
                    // closer instead of switching on at full size.
                    const float urgency =
                        1.0f - smoothstep(config_.brake_distance_m, config_.steer_distance_m,
                                           clearance);
                    common::Vector3 candidate{};
                    candidate.y = escape->y / sideways * config_.max_lateral_offset_m * urgency;
                    candidate.z = escape->z / sideways * config_.max_vertical_offset_m * urgency;

                    const common::Vector3 candidateMap =
                        alignment_.bodyToMap(pose.pose.orientation, candidate);
                    if (!offsetBreaksGeofence(target.position, candidateMap)) {
                        desiredOffset = candidate;
                        if (!have_commitment_ ||
                            (now - committed_at_) >= config_.commit_time) {
                            committed_escape_ = *escape;
                            committed_at_ = now;
                            have_commitment_ = true;
                        }
                    }
                }
            }
        }
    } else {
        have_commitment_ = false;
    }

    // Braking latches whatever offset was already chosen, so that when the way
    // clears the drone resumes in the direction it had committed to rather
    // than starting the decision again from nothing.
    if (proposed == AvoidanceState::Braking && lengthOf(desiredOffset) < 1e-6f) {
        desiredOffset = offset_body_;
    }

    decision.state = settleState(proposed, now);
    decision.offset_body = slewOffset(desiredOffset, now);
    decision.target_offset_map =
        alignment_.bodyToMap(pose.pose.orientation, decision.offset_body);

    decision.cruise_scale =
        config_.enable_slow
            ? clamp01(config_.min_cruise_scale +
                       (1.0f - config_.min_cruise_scale) *
                           smoothstep(config_.brake_distance_m, config_.slow_distance_m, clearance))
            : 1.0f;

    // Ease off the path-centring term in proportion to how far aside the drone
    // is deliberately flying, so the two do not fight.
    const float offsetSpan = std::sqrt(config_.max_lateral_offset_m * config_.max_lateral_offset_m +
                                        config_.max_vertical_offset_m * config_.max_vertical_offset_m);
    const float offsetFraction =
        offsetSpan > 1e-6f ? clamp01(lengthOf(decision.offset_body) / offsetSpan) : 0.0f;
    decision.path_offset_scale = 1.0f - 0.7f * offsetFraction;

    decision.hold = config_.enable_brake && decision.state == AvoidanceState::Braking;

    // --- how long have we been stuck --------------------------------------
    if (decision.state == AvoidanceState::Braking || decision.state == AvoidanceState::Blocked) {
        if (!inBrake) {
            braking_since_ = now;
            best_clearance_while_braking_ = corridor.clearance_m;
        } else if (corridor.clearance_m > best_clearance_while_braking_ + kProgressEpsilonM) {
            // Genuine progress: it is moving out of the way. Restart the clock.
            braking_since_ = now;
            best_clearance_while_braking_ = corridor.clearance_m;
        }
        if ((now - braking_since_) >= config_.blocked_abort_after) {
            blocked_ = true;
            decision.state = AvoidanceState::Blocked;
            decision.hold = true;
        }
    } else {
        blocked_ = false;
        best_clearance_while_braking_ = std::numeric_limits<float>::infinity();
    }

    switch (decision.state) {
        case AvoidanceState::Clear: decision.reason = "clear"; break;
        case AvoidanceState::Slowing: decision.reason = "obstacle ahead"; break;
        case AvoidanceState::Steering: decision.reason = "steering around"; break;
        case AvoidanceState::Braking: decision.reason = "too close to pass"; break;
        case AvoidanceState::Blocked: decision.reason = "blocked"; break;
        default: break;
    }

    last_step_at_ = now;
    have_last_step_ = true;
    last_ = decision;
    return decision;
}

} // namespace control
