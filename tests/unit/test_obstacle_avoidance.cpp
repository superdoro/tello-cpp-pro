#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <random>

#include "control/obstacle_avoidance.hpp"
#include "test_check.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using common::ObstacleSnapshot;
using control::AvoidanceState;

constexpr float kInf = std::numeric_limits<float>::infinity();

// Straight ahead in the map frame. With an identity orientation the frame
// conventions map +z (map) onto +x (body, forward), so a target at (0,0,5) is
// five metres in front of the nose.
const common::Vector3 kTargetAhead{0.0f, 0.0f, 5.0f};

control::AvoidanceConfig enabledConfig() {
    control::AvoidanceConfig config;
    config.enabled = true;
    return config;
}

common::PoseEstimate trackedPose(const common::Vector3& position = {}) {
    common::PoseEstimate pose;
    pose.pose.position = position;
    pose.pose.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    pose.state = common::TrackingState::Ok;
    pose.confidence = 0.9f;
    return pose;
}

control::Waypoint waypointAt(const common::Vector3& position) {
    control::Waypoint waypoint;
    waypoint.position = position;
    return waypoint;
}

// Everything observed, nothing in the way - the picture of an empty room.
ObstacleSnapshot clearSnapshot(Clock::time_point stamp, float confidence = 0.9f) {
    ObstacleSnapshot snapshot;
    snapshot.stamp = stamp;
    snapshot.computed_at = stamp;
    snapshot.health = common::PerceptionHealth::Ok;
    snapshot.depth_scale_confidence = confidence;
    for (auto& bin : snapshot.bins) {
        bin.flags = common::bin_flags::kObserved;
        bin.range_m = kInf;
    }
    return snapshot;
}

void placeObstacle(ObstacleSnapshot& snapshot, int azimuthBin, int elevationBin, float range,
                    std::uint8_t extraFlags = 0) {
    common::ObstacleBin& bin = snapshot.at(azimuthBin, elevationBin);
    bin.range_m = range;
    bin.confidence = 0.9f;
    bin.flags |= common::bin_flags::kObserved | common::bin_flags::kOccupied | extraFlags;
}

bool isIdentity(const control::AvoidanceDecision& d) {
    return d.cruise_scale == 1.0f && d.target_offset_map.x == 0.0f &&
            d.target_offset_map.y == 0.0f && d.target_offset_map.z == 0.0f && !d.hold &&
            d.path_offset_scale == 1.0f;
}

float magnitude(const common::Vector3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// ---------------------------------------------------------------------------

void testDisabledIsExactlyIdentity() {
    test::beginCase("disabled produces a bit-for-bit identity decision");
    // THE property the whole feature rests on. Because the app applies
    // `planner.cruiseScale() * decision.cruise_scale` and `target += offset`,
    // an identity decision means the commands are arithmetically unchanged -
    // a stronger statement than any end-to-end diff, because it holds for
    // every input rather than for one trajectory.
    control::ObstacleAvoidance avoidance;  // enabled defaults to false
    const auto now = Clock::now();

    auto snapshot = clearSnapshot(now);
    placeObstacle(snapshot, 0, 2, 0.2f);  // right in its face; must not matter

    const auto decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
    CHECK(isIdentity(decision));
    CHECK(decision.state == AvoidanceState::Disabled);
    CHECK(!avoidance.blockedTooLong());
}

void testNoSnapshotIsIdentityDuringStartup() {
    test::beginCase("no obstacle data during startup grace is identity, not a slowdown");
    control::ObstacleAvoidance avoidance(enabledConfig());
    const auto now = Clock::now();
    const auto decision =
        avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), std::nullopt, now);
    CHECK(isIdentity(decision));
    CHECK(decision.state == AvoidanceState::Degraded);
}

void testUntrustedPoseIsIdentity() {
    test::beginCase("an untrustworthy pose hands the problem back to the planner");
    control::ObstacleAvoidance avoidance(enabledConfig());
    const auto now = Clock::now();
    auto snapshot = clearSnapshot(now);
    placeObstacle(snapshot, 0, 2, 0.3f);

    common::PoseEstimate lost = trackedPose();
    lost.state = common::TrackingState::Lost;

    const auto decision = avoidance.evaluate(lost, waypointAt(kTargetAhead), snapshot, now);
    CHECK(isIdentity(decision));
    CHECK(decision.state == AvoidanceState::Disabled);
}

void testDegradedInputsSlowButNeverSteer() {
    test::beginCase("stale, unhealthy and low-confidence input all degrade to a speed cap");
    const auto config = enabledConfig();
    const auto now = Clock::now();

    struct Case {
        const char* name;
        ObstacleSnapshot snapshot;
    };
    std::vector<Case> cases;
    {
        auto stale = clearSnapshot(now - std::chrono::milliseconds(900));
        placeObstacle(stale, 0, 2, 0.5f);
        cases.push_back({"stale", stale});

        auto unhealthy = clearSnapshot(now);
        unhealthy.health = common::PerceptionHealth::EngineFailed;
        placeObstacle(unhealthy, 0, 2, 0.5f);
        cases.push_back({"unhealthy", unhealthy});

        auto unscaled = clearSnapshot(now);
        unscaled.health = common::PerceptionHealth::NoDepthScale;
        cases.push_back({"no depth scale", unscaled});

        auto timid = clearSnapshot(now, 0.05f);
        placeObstacle(timid, 0, 2, 0.5f);
        cases.push_back({"low confidence", timid});
    }

    for (const auto& c : cases) {
        control::ObstacleAvoidance avoidance(config);
        const auto decision =
            avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), c.snapshot, now);
        CHECK(decision.state == AvoidanceState::Degraded);
        CHECK(decision.cruise_scale <= 1.0f);
        CHECK_NEAR(decision.cruise_scale, config.degraded_cruise_scale, 1e-6);
        // Never steer on input this weak, and never brake on it either: an
        // obstacle 0.5 m ahead in three of these cases does NOT produce a hold.
        CHECK_EQ(magnitude(decision.target_offset_map), 0.0f);
        CHECK(!decision.hold);
    }
}

void testUnobservedCorridorDegradesRatherThanReadingAsClear() {
    test::beginCase("an unobserved corridor is not treated as an empty one");
    control::ObstacleAvoidance avoidance(enabledConfig());
    const auto now = Clock::now();

    // A perfectly healthy, confident snapshot in which nothing has been seen.
    // Every bin has infinite range, which naively reads as wide-open space.
    ObstacleSnapshot blind;
    blind.stamp = now;
    blind.health = common::PerceptionHealth::Ok;
    blind.depth_scale_confidence = 0.9f;

    const auto decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), blind, now);
    CHECK(decision.state == AvoidanceState::Degraded);
    CHECK(decision.cruise_scale < 1.0f);
    CHECK(!decision.hold);
}

void testClearCorridorFliesAtFullSpeed() {
    test::beginCase("an observed, empty corridor costs nothing");
    control::ObstacleAvoidance avoidance(enabledConfig());
    const auto now = Clock::now();
    const auto decision =
        avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), clearSnapshot(now), now);
    CHECK(decision.state == AvoidanceState::Clear);
    CHECK(isIdentity(decision));
}

void testObstacleAheadSlowsThenBrakes() {
    test::beginCase("cruise falls as an obstacle closes, and a hold arrives at brake distance");
    const auto config = enabledConfig();
    const auto start = Clock::now();

    float previousScale = 1.0f;
    for (int i = 0; i < 5; ++i) {
        control::ObstacleAvoidance avoidance(config);
        const float range = 4.5f - static_cast<float>(i) * 0.9f;  // 4.5 .. 0.9
        auto snapshot = clearSnapshot(start);
        placeObstacle(snapshot, 0, 2, range);

        const auto decision =
            avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, start);
        CHECK(decision.cruise_scale <= previousScale + 1e-6f);
        CHECK(decision.cruise_scale >= config.min_cruise_scale - 1e-6f);
        previousScale = decision.cruise_scale;

        if (range <= config.brake_distance_m) {
            CHECK(decision.hold);
            CHECK(decision.state == AvoidanceState::Braking);
        } else {
            CHECK(!decision.hold);
        }
    }
}

void testDisablingALayerDisablesOnlyThatLayer() {
    test::beginCase("each layer can be switched off independently");
    const auto now = Clock::now();
    auto snapshot = clearSnapshot(now);
    placeObstacle(snapshot, 0, 2, 0.6f);  // inside brake distance

    {
        auto config = enabledConfig();
        config.enable_brake = false;
        control::ObstacleAvoidance avoidance(config);
        const auto d = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
        CHECK(!d.hold);  // the conservative first-flight setting
    }
    {
        auto config = enabledConfig();
        config.enable_slow = false;
        control::ObstacleAvoidance avoidance(config);
        const auto d = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
        CHECK_EQ(d.cruise_scale, 1.0f);
        CHECK(d.hold);  // but braking still works
    }
    {
        auto config = enabledConfig();
        config.enable_steer = false;
        control::ObstacleAvoidance avoidance(config);
        const auto d = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
        CHECK_EQ(magnitude(d.target_offset_map), 0.0f);
    }
}

void testSteeringPicksAClearObservedSide() {
    test::beginCase("steering aims at a side that is both clear and observed");
    control::ObstacleAvoidance avoidance(enabledConfig());
    auto now = Clock::now();

    // A wall dead ahead at 1.6 m, spanning the middle bins. Both flanks clear.
    auto snapshot = clearSnapshot(now);
    for (int az : {23, 0, 1}) placeObstacle(snapshot, az, 2, 1.6f);

    // Let the slew limiter accumulate: one step cannot produce a full offset,
    // which is itself the point of the limiter.
    control::AvoidanceDecision decision;
    for (int i = 0; i < 60; ++i) {
        now += std::chrono::milliseconds(33);
        auto fresh = snapshot;
        fresh.stamp = now;
        decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), fresh, now);
    }

    CHECK(decision.state == AvoidanceState::Steering);
    CHECK(magnitude(decision.offset_body) > 0.05f);
    // It must have gone sideways or vertically, never forwards.
    CHECK_EQ(decision.offset_body.x, 0.0f);
    CHECK(std::abs(decision.offset_body.y) <= enabledConfig().max_lateral_offset_m + 1e-5f);
    CHECK(std::abs(decision.offset_body.z) <= enabledConfig().max_vertical_offset_m + 1e-5f);
    // Path centring eases off so it does not fight the manoeuvre.
    CHECK(decision.path_offset_scale < 1.0f);
    CHECK(decision.path_offset_scale >= 0.3f);
}

void testNeverSteersIntoUnobservedSpace() {
    test::beginCase("an unobserved flank is never chosen as an escape");
    control::ObstacleAvoidance avoidance(enabledConfig());
    auto now = Clock::now();

    // Wall ahead. The left flank is observed and clear; the right flank is
    // simply unknown - which reads as infinite range unless Observed is
    // checked. This is the single most safety-relevant rejection in the class.
    auto snapshot = clearSnapshot(now);
    for (int az : {23, 0, 1}) placeObstacle(snapshot, az, 2, 1.6f);
    for (int el = 0; el < ObstacleSnapshot::kElevationBins; ++el) {
        for (int az : {20, 21, 22}) {  // the right side: -60..-30 degrees
            snapshot.at(az, el) = common::ObstacleBin{};  // unknown
        }
    }

    control::AvoidanceDecision decision;
    for (int i = 0; i < 60; ++i) {
        now += std::chrono::milliseconds(33);
        auto fresh = snapshot;
        fresh.stamp = now;
        decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), fresh, now);
    }

    // Body +y is left. It must have chosen left, where it can actually see.
    CHECK(decision.offset_body.y > 0.0f);
}

void testDynamicObstaclesAreNotDestinations() {
    test::beginCase("a person is an obstacle to avoid, never a gap to aim at");
    control::ObstacleAvoidance avoidance(enabledConfig());
    auto now = Clock::now();

    auto snapshot = clearSnapshot(now);
    for (int az : {23, 0, 1}) placeObstacle(snapshot, az, 2, 1.6f);
    // The right flank is "clear" at long range but flagged dynamic: something
    // is moving through it. Committing 1.5 s to fly there is wrong even though
    // the range says it is free.
    for (int el = 0; el < ObstacleSnapshot::kElevationBins; ++el) {
        for (int az : {20, 21, 22}) {
            placeObstacle(snapshot, az, el, 6.0f, common::bin_flags::kDynamic);
        }
    }

    control::AvoidanceDecision decision;
    for (int i = 0; i < 60; ++i) {
        now += std::chrono::milliseconds(33);
        auto fresh = snapshot;
        fresh.stamp = now;
        decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), fresh, now);
    }
    CHECK(decision.offset_body.y > 0.0f);  // went left, away from the moving thing
}

void testOffsetNeverBreaksTheGeofence() {
    test::beginCase("an escape that would leave the flight volume is refused");
    auto config = enabledConfig();
    control::GeofenceConfig fence;
    fence.max_radius_m = 5.0f;  // the target sits exactly on it
    control::ObstacleAvoidance avoidance(config);
    avoidance.setGeofence(fence);

    auto now = Clock::now();
    auto snapshot = clearSnapshot(now);
    for (int az : {23, 0, 1}) placeObstacle(snapshot, az, 2, 1.6f);

    control::AvoidanceDecision decision;
    for (int i = 0; i < 60; ++i) {
        now += std::chrono::milliseconds(33);
        auto fresh = snapshot;
        fresh.stamp = now;
        decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), fresh, now);
    }
    // Any offset at all would push the target past 5 m from the origin, and
    // the planner would abort on the very next pose.
    CHECK_EQ(magnitude(decision.offset_body), 0.0f);
    // It still slows and brakes, though - the fence removes an option, not
    // the whole layer.
    CHECK(decision.cruise_scale < 1.0f);
}

void testOffsetRespectsTheSlewLimit() {
    test::beginCase("the offset never steps further in one frame than the slew limit allows");
    const auto config = enabledConfig();
    control::ObstacleAvoidance avoidance(config);
    auto now = Clock::now();

    auto snapshot = clearSnapshot(now);
    for (int az : {23, 0, 1}) placeObstacle(snapshot, az, 2, 1.6f);

    common::Vector3 previous{};
    for (int i = 0; i < 120; ++i) {
        now += std::chrono::milliseconds(33);
        auto fresh = snapshot;
        fresh.stamp = now;
        // Halfway through, the obstacle vanishes - the offset must decay just
        // as gently as it grew.
        if (i == 60) snapshot = clearSnapshot(now);

        const auto decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), fresh, now);
        const common::Vector3 step{decision.offset_body.x - previous.x,
                                    decision.offset_body.y - previous.y,
                                    decision.offset_body.z - previous.z};
        const float allowed = config.offset_rate_m_per_s * 0.033f + 1e-4f;
        CHECK(magnitude(step) <= allowed);
        // Well under the PID's "this is a new setpoint" threshold, which is
        // what stops a manoeuvre becoming a one-frame stick slam.
        CHECK(magnitude(step) < 0.75f);
        previous = decision.offset_body;
    }
}

void testHysteresisStopsStateChatter() {
    test::beginCase("a clearance oscillating across a threshold does not chatter");
    const auto config = enabledConfig();
    control::ObstacleAvoidance avoidance(config);
    auto now = Clock::now();

    int transitions = 0;
    AvoidanceState previous = AvoidanceState::Disabled;
    for (int i = 0; i < 200; ++i) {
        now += std::chrono::milliseconds(33);
        // Straddle the brake threshold every single frame.
        const float range = config.brake_distance_m + ((i % 2 == 0) ? -0.05f : 0.05f);
        auto snapshot = clearSnapshot(now);
        placeObstacle(snapshot, 0, 2, range);

        const auto decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
        if (decision.state != previous) {
            ++transitions;
            previous = decision.state;
        }
    }
    // Without the Schmitt margin and dwell this would be ~200. Allow a couple
    // for settling in.
    CHECK(transitions <= 3);
}

void testEscapeSideIsCommittedTo() {
    test::beginCase("a symmetric obstacle does not make the drone weave between sides");
    const auto config = enabledConfig();
    control::ObstacleAvoidance avoidance(config);
    auto now = Clock::now();

    auto snapshot = clearSnapshot(now);
    for (int az : {23, 0, 1}) placeObstacle(snapshot, az, 2, 1.6f);

    int signChanges = 0;
    float previousSign = 0.0f;
    for (int i = 0; i < 45; ++i) {  // ~1.5 s, the commit window
        now += std::chrono::milliseconds(33);
        auto fresh = snapshot;
        fresh.stamp = now;
        const auto decision = avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), fresh, now);
        const float sign = decision.offset_body.y > 0.01f    ? 1.0f
                            : decision.offset_body.y < -0.01f ? -1.0f
                                                              : 0.0f;
        if (sign != 0.0f && previousSign != 0.0f && sign != previousSign) ++signChanges;
        if (sign != 0.0f) previousSign = sign;
    }
    CHECK_EQ(signChanges, 0);
}

void testBlockedFiresOnlyAfterTheConfiguredTime() {
    test::beginCase("blockedTooLong fires at its deadline, and not before");
    auto config = enabledConfig();
    config.blocked_abort_after = std::chrono::milliseconds(2000);
    control::ObstacleAvoidance avoidance(config);
    auto now = Clock::now();

    bool firedEarly = false;
    for (int i = 0; i < 50; ++i) {  // 50 * 33 ms = 1.65 s, short of the deadline
        now += std::chrono::milliseconds(33);
        auto snapshot = clearSnapshot(now);
        placeObstacle(snapshot, 0, 2, 0.5f);
        avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
        if (avoidance.blockedTooLong()) firedEarly = true;
    }
    CHECK(!firedEarly);

    for (int i = 0; i < 20; ++i) {
        now += std::chrono::milliseconds(33);
        auto snapshot = clearSnapshot(now);
        placeObstacle(snapshot, 0, 2, 0.5f);
        avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
    }
    CHECK(avoidance.blockedTooLong());
    CHECK(avoidance.state() == AvoidanceState::Blocked);
}

void testClearingObstacleRestartsTheBlockedClock() {
    test::beginCase("something moving out of the way is not 'blocked'");
    auto config = enabledConfig();
    config.blocked_abort_after = std::chrono::milliseconds(1500);
    control::ObstacleAvoidance avoidance(config);
    auto now = Clock::now();

    for (int i = 0; i < 120; ++i) {
        now += std::chrono::milliseconds(33);
        // Backing away steadily: never blocked, however long it takes.
        const float range = 0.5f + static_cast<float>(i) * 0.004f;
        auto snapshot = clearSnapshot(now);
        placeObstacle(snapshot, 0, 2, range);
        avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), snapshot, now);
        CHECK(!avoidance.blockedTooLong());
    }
}

void testRequirePerceptionReportsSustainedFaults() {
    test::beginCase("require_perception reports a sustained fault, and only a sustained one");
    auto config = enabledConfig();
    config.require_perception = true;
    control::ObstacleAvoidance avoidance(config);
    auto now = Clock::now();

    auto broken = clearSnapshot(now);
    broken.health = common::PerceptionHealth::EngineFailed;
    avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), broken, now);
    CHECK(!avoidance.unhealthyTooLong());

    now += std::chrono::seconds(3);
    broken.stamp = now;
    avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), broken, now);
    CHECK(avoidance.unhealthyTooLong());

    // Recovery clears it.
    now += std::chrono::milliseconds(33);
    avoidance.evaluate(trackedPose(), waypointAt(kTargetAhead), clearSnapshot(now), now);
    CHECK(!avoidance.unhealthyTooLong());
}

void testNeverFasterThanTodayUnderFuzz() {
    test::beginCase("100k random inputs never produce a speed-up or an oversized offset");
    // The "cannot make the drone less safe" invariant, mechanised. Any input
    // at all - garbage snapshots, absurd ranges, tumbling poses - must leave
    // cruise_scale in [0,1] and the offset inside its configured caps.
    const auto config = enabledConfig();
    control::ObstacleAvoidance avoidance(config);
    std::mt19937 rng(20260912);
    std::uniform_real_distribution<float> unitFloat(0.0f, 1.0f);
    std::uniform_real_distribution<float> signedFloat(-1.0f, 1.0f);
    std::uniform_int_distribution<int> smallInt(0, 7);

    auto now = Clock::now();
    int violations = 0;

    for (int iteration = 0; iteration < 100000; ++iteration) {
        now += std::chrono::milliseconds(1 + smallInt(rng) * 10);

        ObstacleSnapshot snapshot;
        snapshot.stamp = now - std::chrono::milliseconds(smallInt(rng) * 200);
        snapshot.health = static_cast<common::PerceptionHealth>(smallInt(rng) % 7);
        snapshot.depth_scale_confidence = unitFloat(rng);
        for (auto& bin : snapshot.bins) {
            bin.flags = static_cast<std::uint8_t>(smallInt(rng) * 2);
            // Ranges from "touching the lens" to nonsense, plus infinities.
            bin.range_m = unitFloat(rng) < 0.1f ? kInf : unitFloat(rng) * 30.0f - 2.0f;
            bin.confidence = unitFloat(rng);
        }

        common::PoseEstimate pose;
        pose.pose.position = {signedFloat(rng) * 10.0f, signedFloat(rng) * 10.0f,
                               signedFloat(rng) * 10.0f};
        pose.pose.orientation = common::normalize({signedFloat(rng), signedFloat(rng),
                                                    signedFloat(rng), signedFloat(rng)});
        pose.state = (smallInt(rng) < 6) ? common::TrackingState::Ok : common::TrackingState::Lost;
        pose.confidence = unitFloat(rng);

        const control::Waypoint target = waypointAt(
            {signedFloat(rng) * 10.0f, signedFloat(rng) * 10.0f, signedFloat(rng) * 10.0f});

        avoidance.setSpeed(unitFloat(rng) * 4.0f);
        const auto d = avoidance.evaluate(pose, target, snapshot, now);

        const bool ok = std::isfinite(d.cruise_scale) && d.cruise_scale >= 0.0f &&
                         d.cruise_scale <= 1.0f && std::isfinite(d.path_offset_scale) &&
                         d.path_offset_scale >= 0.0f && d.path_offset_scale <= 1.0f &&
                         d.offset_body.x == 0.0f &&
                         std::abs(d.offset_body.y) <= config.max_lateral_offset_m + 1e-4f &&
                         std::abs(d.offset_body.z) <= config.max_vertical_offset_m + 1e-4f &&
                         std::isfinite(magnitude(d.target_offset_map));
        if (!ok) ++violations;
    }
    CHECK_EQ(violations, 0);
}

}  // namespace

int main() {
    std::cout << "obstacle avoidance\n";
    testDisabledIsExactlyIdentity();
    testNoSnapshotIsIdentityDuringStartup();
    testUntrustedPoseIsIdentity();
    testDegradedInputsSlowButNeverSteer();
    testUnobservedCorridorDegradesRatherThanReadingAsClear();
    testClearCorridorFliesAtFullSpeed();
    testObstacleAheadSlowsThenBrakes();
    testDisablingALayerDisablesOnlyThatLayer();
    testSteeringPicksAClearObservedSide();
    testNeverSteersIntoUnobservedSpace();
    testDynamicObstaclesAreNotDestinations();
    testOffsetNeverBreaksTheGeofence();
    testOffsetRespectsTheSlewLimit();
    testHysteresisStopsStateChatter();
    testEscapeSideIsCommittedTo();
    testBlockedFiresOnlyAfterTheConfiguredTime();
    testClearingObstacleRestartsTheBlockedClock();
    testRequirePerceptionReportsSustainedFaults();
    testNeverFasterThanTodayUnderFuzz();
    return test::summary("obstacle avoidance");
}
