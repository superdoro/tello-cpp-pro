// The planner is where every "stop flying" decision lives. All of it is
// reachable without a drone, so all of it is tested here - these are the
// checks that would otherwise cost a crashed airframe to discover.
#include <algorithm>
#include <chrono>
#include <cmath>

#include "control/mission_planner.hpp"
#include "test_check.hpp"

using namespace std::chrono_literals;

namespace {

common::PoseEstimate poseAt(float x, float y, float z,
                             common::TrackingState state = common::TrackingState::Ok,
                             float confidence = 1.0f) {
    common::PoseEstimate estimate;
    estimate.pose.position = {x, y, z};
    estimate.pose.orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    estimate.state = state;
    estimate.confidence = confidence;
    return estimate;
}

control::Mission twoWaypointMission() {
    control::Mission mission;
    mission.name = "test";
    // These cases are about arrival, dwell and the safety paths, so they use
    // the settle-at-each-waypoint behaviour explicitly.
    mission.config.continuous = false;

    control::Waypoint first;
    first.position = {0.0f, 0.0f, 1.0f};
    first.position_tolerance_m = 0.2f;
    first.dwell = 0ms;
    first.timeout = 10s;

    control::Waypoint second = first;
    second.position = {0.0f, 0.0f, 2.0f};

    mission.waypoints = {first, second};
    return mission;
}

void testAdvancesThroughWaypoints() {
    test::beginCase("reaching a waypoint advances to the next, then completes");
    control::MissionPlanner planner;
    planner.loadMission(twoWaypointMission());
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    CHECK(planner.status() == control::MissionStatus::Running);
    CHECK_EQ(planner.currentIndex(), 0u);

    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);
    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);
    CHECK_EQ(planner.currentIndex(), 1u);

    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 2.0f), now);
    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 2.0f), now);
    CHECK(planner.status() == control::MissionStatus::Complete);
}

void testDwellPreventsFlythrough() {
    test::beginCase("a brief pass through the tolerance sphere is not an arrival");
    control::Mission mission = twoWaypointMission();
    mission.waypoints[0].dwell = 500ms;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);
    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);
    CHECK_EQ(planner.currentIndex(), 0u);  // dwell not satisfied yet

    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);  // flew past
    now += 600ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    CHECK_EQ(planner.currentIndex(), 0u);  // and the dwell clock restarted
}

void testLostTrackingHoldsThenAborts() {
    test::beginCase("lost tracking holds, and aborts once the limit passes");
    control::Mission mission = twoWaypointMission();
    mission.config.max_tracking_loss = 1s;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    CHECK(planner.status() == control::MissionStatus::Running);

    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f, common::TrackingState::Lost), now);
    CHECK(planner.status() == control::MissionStatus::Holding);

    now += 500ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f, common::TrackingState::Lost), now);
    CHECK(planner.status() == control::MissionStatus::Holding);

    now += 800ms;  // now past max_tracking_loss
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f, common::TrackingState::Lost), now);
    CHECK(planner.status() == control::MissionStatus::Aborted);
}

void testRecoveryResumes() {
    test::beginCase("tracking coming back resumes the mission");
    control::MissionPlanner planner;
    planner.loadMission(twoWaypointMission());
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f, common::TrackingState::Lost), now);
    CHECK(planner.status() == control::MissionStatus::Holding);

    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    CHECK(planner.status() == control::MissionStatus::Running);
}

void testLowConfidenceCountsAsUntracked() {
    test::beginCase("a nominally-OK pose with too few inliers is not trusted");
    control::Mission mission = twoWaypointMission();
    mission.config.min_confidence = 0.5f;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f, common::TrackingState::Ok, 0.1f), now);
    CHECK(planner.status() == control::MissionStatus::Holding);
}

void testGeofenceAborts() {
    test::beginCase("a pose outside the geofence aborts immediately");
    control::Mission mission = twoWaypointMission();
    mission.config.geofence.max_radius_m = 5.0f;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 50.0f), now);
    CHECK(planner.status() == control::MissionStatus::Aborted);
    CHECK(!planner.abortReason().empty());
}

void testAltitudeGeofenceUsesMapUp() {
    test::beginCase("altitude limits are measured along the map's up axis");
    control::Mission mission = twoWaypointMission();
    mission.config.geofence.max_radius_m = 100.0f;
    mission.config.geofence.max_altitude_m = 2.0f;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    // Default map up is -y, so a large NEGATIVE y is a high altitude.
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, -9.0f, 0.0f), now);
    CHECK(planner.status() == control::MissionStatus::Aborted);
}

void testWaypointTimeoutAborts() {
    test::beginCase("failing to reach a waypoint in time aborts");
    control::Mission mission = twoWaypointMission();
    mission.waypoints[0].timeout = 1s;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    now += 2s;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    CHECK(planner.status() == control::MissionStatus::Aborted);
}

void testEmptyMissionAborts() {
    test::beginCase("starting an empty mission aborts rather than completing");
    control::MissionPlanner planner;
    planner.loadMission(control::Mission{});
    planner.start();
    CHECK(planner.status() == control::MissionStatus::Aborted);
}

// Continuous mode is what stops a route feeling like a series of hops. These
// pin down that it keeps moving, keeps aiming ahead, and still lands cleanly
// on the last waypoint.
control::Mission straightLineMission(bool continuous) {
    control::Mission mission;
    mission.config.continuous = continuous;
    mission.config.lookahead_m = 1.0f;
    mission.config.geofence.max_radius_m = 100.0f;
    mission.config.geofence.max_altitude_m = 100.0f;
    mission.config.geofence.min_altitude_m = -100.0f;

    for (int i = 1; i <= 5; ++i) {
        control::Waypoint waypoint;
        waypoint.position = {0.0f, 0.0f, static_cast<float>(i)};  // 1m apart along +z
        waypoint.position_tolerance_m = 0.2f;
        waypoint.dwell = 0ms;
        waypoint.timeout = 60s;
        mission.waypoints.push_back(waypoint);
    }
    return mission;
}

void testContinuousAimsAheadOfTheNearestWaypoint() {
    test::beginCase("continuous mode targets a point ahead on the path, not the waypoint");
    control::MissionPlanner planner;
    planner.loadMission(straightLineMission(true));
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);

    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    // 1m of lookahead from the origin along the route.
    CHECK_NEAR(target->position.z, 1.0, 0.01);
}

void testContinuousDoesNotStallAtAWaypoint() {
    test::beginCase("flying through a waypoint advances without dwelling");
    control::MissionPlanner planner;
    planner.loadMission(straightLineMission(true));
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);  // exactly on waypoint 1
    CHECK_EQ(planner.currentIndex(), 1u);                  // already moved on

    // And the target is still ahead, so the controller keeps commanding motion.
    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    CHECK(target->position.z > 1.5f);
}

void testContinuousSkipsWaypointsAlreadyPassed() {
    test::beginCase("cutting a corner does not leave the planner behind");
    control::MissionPlanner planner;
    planner.loadMission(straightLineMission(true));
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    // Well past waypoints 1-3 without ever being within tolerance of them.
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 3.4f), now);
    CHECK(planner.currentIndex() >= 3u);
}

void testContinuousStillStopsAtTheLastWaypoint() {
    test::beginCase("the final waypoint is a real arrival, not a flythrough");
    control::Mission mission = straightLineMission(true);
    mission.waypoints.back().dwell = 200ms;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 4.9f), now);
    CHECK(planner.status() == control::MissionStatus::Running);

    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    CHECK(planner.status() != control::MissionStatus::Complete);  // dwell not met

    now += 300ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    CHECK(planner.status() == control::MissionStatus::Complete);
}

void testStopAtWaypointsStillWorks() {
    test::beginCase("stop-at-waypoints mode keeps the old settle-then-go behaviour");
    control::Mission mission = straightLineMission(false);
    mission.waypoints[0].dwell = 200ms;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);
    CHECK_EQ(planner.currentIndex(), 0u);  // must dwell first
    now += 300ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.0f), now);
    CHECK_EQ(planner.currentIndex(), 1u);
}

void testPassingWaypointsKeepsResettingTheTimeout() {
    test::beginCase("a long route does not time out while it is making progress");
    control::Mission mission = straightLineMission(true);
    // Every waypoint allows 10s, and the whole flight below takes 40s. If the
    // clock is not restarted as waypoints go by, the per-waypoint timeout
    // silently becomes a whole-mission one and aborts mid-route - which is
    // exactly what happened on the first real flight.
    for (auto& waypoint : mission.waypoints) waypoint.timeout = 10s;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    for (int step = 1; step <= 5; ++step) {
        now += 8s;  // slow, but progressing
        planner.onPoseUpdate(poseAt(0.0f, 0.0f, static_cast<float>(step)), now);
        CHECK(planner.status() != control::MissionStatus::Aborted);
    }
    // Arrival at the final waypoint needs a second in-tolerance frame to
    // close out the dwell window - 33ms in flight, one more call here.
    now += 100ms;
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    CHECK(planner.status() == control::MissionStatus::Complete);
}

void testStallingStillTimesOut() {
    test::beginCase("making no progress still aborts");
    control::Mission mission = straightLineMission(true);
    for (auto& waypoint : mission.waypoints) waypoint.timeout = 5s;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    now += 10s;  // pinned against something, never advancing
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    CHECK(planner.status() == control::MissionStatus::Aborted);
}

void testLookaheadDoesNotAimThroughACorner() {
    test::beginCase("the lookahead stops at a bend instead of cutting across it");
    control::Mission mission;
    mission.config.continuous = true;
    mission.config.lookahead_m = 3.0f;  // deliberately longer than the first leg
    mission.config.lookahead_corner_limit_rad = 0.6f;
    mission.config.geofence.max_radius_m = 100.0f;
    mission.config.geofence.max_altitude_m = 100.0f;
    mission.config.geofence.min_altitude_m = -100.0f;

    // An L: two metres along +z, then a right-angle turn along +x. Cutting
    // that corner is what flies a drone into the wall on the inside of the
    // bend.
    const float corners[4][3] = {{0, 0, 1}, {0, 0, 2}, {1, 0, 2}, {2, 0, 2}};
    for (const auto& c : corners) {
        control::Waypoint waypoint;
        waypoint.position = {c[0], c[1], c[2]};
        waypoint.position_tolerance_m = 0.2f;
        waypoint.dwell = 0ms;
        waypoint.timeout = 60s;
        mission.waypoints.push_back(waypoint);
    }

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);

    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    // Without the corner limit the 3m lookahead would land around (1,0,2),
    // a diagonal straight through the inside of the bend. It must instead
    // aim at the corner itself.
    CHECK_NEAR(target->position.x, 0.0, 0.01);
    CHECK_NEAR(target->position.z, 2.0, 0.01);
}

void testLookaheadIsUnrestrictedOnAStraight() {
    test::beginCase("a straight route still gets the full lookahead");
    control::Mission mission = straightLineMission(true);
    mission.config.lookahead_m = 2.5f;
    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);

    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    CHECK_NEAR(target->position.z, 2.5, 0.01);
}

void testLookaheadGrowsWithSpeed() {
    test::beginCase("the lookahead lengthens as the drone speeds up");
    control::Mission mission = straightLineMission(true);
    mission.config.lookahead_m = 1.0f;
    mission.config.lookahead_time_s = 0.5f;
    mission.config.lookahead_max_m = 3.0f;
    // A long straight so the lookahead is never truncated by the route end.
    mission.waypoints.clear();
    for (int i = 1; i <= 40; ++i) {
        control::Waypoint waypoint;
        waypoint.position = {0.0f, 0.0f, static_cast<float>(i) * 0.5f};
        waypoint.position_tolerance_m = 0.2f;
        waypoint.dwell = 0ms;
        waypoint.timeout = 600s;
        mission.waypoints.push_back(waypoint);
    }

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 0.0f), now);
    const float slowLookahead = planner.lookahead();
    CHECK_NEAR(slowLookahead, 1.0, 0.01);  // stationary: just the base

    // 2 m/s for a few seconds, long enough for the smoothing to catch up.
    float z = 0.0f;
    for (int step = 0; step < 200; ++step) {
        now += 50ms;
        z += 0.1f;
        planner.onPoseUpdate(poseAt(0.0f, 0.0f, z), now);
    }
    CHECK_NEAR(planner.speed(), 2.0, 0.2);
    CHECK(planner.lookahead() > slowLookahead);
    CHECK(planner.lookahead() <= mission.config.lookahead_max_m);
}

// A 90-degree bend, with enough straight before it to build up speed.
control::Mission missionWithCorner() {
    control::Mission mission;
    mission.config.continuous = true;
    mission.config.lookahead_m = 1.0f;
    mission.config.lookahead_time_s = 0.0f;
    mission.config.corner_brake_time_s = 1.5f;
    mission.config.corner_min_cruise_scale = 0.25f;
    mission.config.geofence.max_radius_m = 100.0f;
    mission.config.geofence.max_altitude_m = 100.0f;
    mission.config.geofence.min_altitude_m = -100.0f;

    const auto add = [&mission](float x, float z) {
        control::Waypoint waypoint;
        waypoint.position = {x, 0.0f, z};
        waypoint.position_tolerance_m = 0.2f;
        waypoint.dwell = 0ms;
        waypoint.timeout = 600s;
        mission.waypoints.push_back(waypoint);
    };
    for (int i = 1; i <= 20; ++i) add(0.0f, static_cast<float>(i) * 0.5f);   // straight up +z
    for (int i = 1; i <= 20; ++i) add(static_cast<float>(i) * 0.5f, 10.0f);  // then along +x
    return mission;
}

// Flies the drone up the straight at a steady 2 m/s and returns the LOWEST
// cruise scale seen from `metresBeforeCorner` onwards. The minimum over the
// approach is the meaningful quantity - whether the drone braked at all -
// rather than whatever value happens to land on the last sample.
float cruiseScaleAt(float metresBeforeCorner) {
    control::MissionPlanner planner;
    planner.loadMission(missionWithCorner());
    planner.start();

    auto now = std::chrono::steady_clock::now();
    float z = 0.0f;
    float lowest = 1.0f;
    // 2 m/s, 20Hz.
    while (z < 10.0f - metresBeforeCorner) {
        now += 50ms;
        z += 0.1f;
        planner.onPoseUpdate(poseAt(0.0f, 0.0f, z), now);
        lowest = std::min(lowest, planner.cruiseScale());
    }
    return lowest;
}

void testCruiseScaleIsFullOnAStraight() {
    test::beginCase("no braking while the route ahead is straight");
    // 6m short of the bend, at 2 m/s the braking window is 3m - the corner is
    // not in it yet.
    CHECK_NEAR(cruiseScaleAt(6.0f), 1.0, 0.01);
}

void testCruiseScaleDropsApproachingACorner() {
    test::beginCase("the cruise term is cut back as a bend comes up");
    const float far = cruiseScaleAt(6.0f);
    const float near = cruiseScaleAt(0.5f);
    CHECK(near < far);
    CHECK(near <= 0.6f);   // meaningfully slowed
    CHECK(near >= 0.25f);  // but never stopped dead
}

void testCruiseScaleRespectsItsFloor() {
    test::beginCase("braking never takes the cruise term below its floor");
    for (float d = 3.0f; d > 0.0f; d -= 0.5f) {
        CHECK(cruiseScaleAt(d) >= 0.25f - 1e-4f);
    }
}

void testPathOffsetIsZeroOnTheRoute() {
    test::beginCase("a drone on the route reports no offset from it");
    control::MissionPlanner planner;
    planner.loadMission(straightLineMission(true));
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 1.5f), now);  // on the line

    const auto offset = planner.pathOffset();
    CHECK_NEAR(std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z),
                0.0, 0.05);
}

void testPathOffsetPointsBackToTheRoute() {
    test::beginCase("a drone beside the route is told which way back to it");
    control::MissionPlanner planner;
    planner.loadMission(straightLineMission(true));
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    // The route runs along +z at x=0; sit 0.6m to the +x side of it.
    planner.onPoseUpdate(poseAt(0.6f, 0.0f, 1.5f), now);

    const auto offset = planner.pathOffset();
    CHECK_NEAR(offset.x, -0.6, 0.05);  // points back towards x=0
    CHECK_NEAR(offset.y, 0.0, 0.05);
    CHECK_NEAR(offset.z, 0.0, 0.1);
}

// --- Reversing and repeating ---------------------------------------------

// Teleports the drone onto its own target until the current pass ends, then
// stops.
//
// Stopping at the PASS boundary is the whole point: between passes the status
// stays Running, so a helper that ran until Complete would quietly fly the
// entire repeat in one call and every assertion about pass transitions would
// be vacuously true.
void flyOnePass(control::MissionPlanner& planner,
                 std::chrono::steady_clock::time_point& now) {
    const int startingPass = planner.currentPass();
    for (std::size_t step = 0; step < 200; ++step) {
        const auto target = planner.currentTarget();
        if (!target.has_value()) break;
        now += 200ms;
        planner.onPoseUpdate(poseAt(target->position.x, target->position.y, target->position.z),
                              now);
        if (planner.status() == control::MissionStatus::Complete ||
            planner.status() == control::MissionStatus::Aborted) {
            break;
        }
        if (planner.currentPass() != startingPass) break;
    }
}

void testReverseFliesTheRouteBackwards() {
    test::beginCase("a reversed pass starts at the far end of the route");
    control::Mission mission = straightLineMission(true);  // waypoints at z = 1..5
    mission.config.reverse = true;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    CHECK(planner.passIsReversed());

    const auto now = std::chrono::steady_clock::now();
    // Starting from the far end, the first target should be back down the
    // route, not up it.
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    CHECK(target->position.z < 5.0f);
}

void testReverseKeepsRecordedHeadingsByDefault() {
    test::beginCase("reversing keeps the mapped camera headings");
    control::Mission mission = straightLineMission(true);
    for (auto& waypoint : mission.waypoints) {
        waypoint.hold_heading = true;
        waypoint.heading_rad = 0.4f;
    }
    mission.config.reverse = true;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    // Unchanged: the camera must keep facing the views the map holds.
    CHECK_NEAR(target->heading_rad, 0.4, 0.01);
}

void testReverseHeadingsTurnsTheDroneAround() {
    test::beginCase("--reverse-headings turns the drone to face its travel");
    control::Mission mission = straightLineMission(true);
    for (auto& waypoint : mission.waypoints) {
        waypoint.hold_heading = true;
        waypoint.heading_rad = 0.4f;
    }
    mission.config.reverse = true;
    mission.config.reverse_headings = true;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    const auto now = std::chrono::steady_clock::now();
    planner.onPoseUpdate(poseAt(0.0f, 0.0f, 5.0f), now);
    const auto target = planner.currentTarget();
    CHECK(target.has_value());
    CHECK_NEAR(std::abs(control::wrapAngle(target->heading_rad - 0.4f)), M_PI, 0.01);
}

void testRepeatKeepsFlyingUntilThePassesAreDone() {
    test::beginCase("repeat starts a new pass instead of completing");
    control::Mission mission = straightLineMission(true);
    mission.config.passes = 3;
    mission.config.ping_pong = true;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();
    CHECK_EQ(planner.currentPass(), 1);
    CHECK_EQ(planner.totalPasses(), 3);

    auto now = std::chrono::steady_clock::now();
    flyOnePass(planner, now);
    CHECK(planner.status() != control::MissionStatus::Complete);
    CHECK(planner.currentPass() >= 2);
}

void testRepeatEventuallyCompletes() {
    test::beginCase("the mission does finish after the last pass");
    control::Mission mission = straightLineMission(true);
    mission.config.passes = 2;
    mission.config.ping_pong = true;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();

    auto now = std::chrono::steady_clock::now();
    for (int pass = 0; pass < 4 && planner.status() == control::MissionStatus::Running; ++pass) {
        flyOnePass(planner, now);
    }
    CHECK(planner.status() == control::MissionStatus::Complete);
    CHECK_EQ(planner.currentPass(), 2);
}

void testPingPongAlternatesDirection() {
    test::beginCase("ping-pong turns the route round on every other pass");
    control::Mission mission = straightLineMission(true);
    mission.config.passes = 3;
    mission.config.ping_pong = true;

    control::MissionPlanner planner;
    planner.loadMission(mission);
    planner.start();
    CHECK(!planner.passIsReversed());  // pass 1 forward

    auto now = std::chrono::steady_clock::now();
    flyOnePass(planner, now);
    CHECK_EQ(planner.currentPass(), 2);
    CHECK(planner.passIsReversed());   // pass 2 back

    flyOnePass(planner, now);
    CHECK_EQ(planner.currentPass(), 3);
    CHECK(!planner.passIsReversed());  // pass 3 forward again
}

}  // namespace

int main() {
    std::cout << "mission_planner\n";
    testAdvancesThroughWaypoints();
    testDwellPreventsFlythrough();
    testLostTrackingHoldsThenAborts();
    testRecoveryResumes();
    testLowConfidenceCountsAsUntracked();
    testGeofenceAborts();
    testAltitudeGeofenceUsesMapUp();
    testWaypointTimeoutAborts();
    testEmptyMissionAborts();
    testContinuousAimsAheadOfTheNearestWaypoint();
    testContinuousDoesNotStallAtAWaypoint();
    testContinuousSkipsWaypointsAlreadyPassed();
    testContinuousStillStopsAtTheLastWaypoint();
    testStopAtWaypointsStillWorks();
    testPassingWaypointsKeepsResettingTheTimeout();
    testStallingStillTimesOut();
    testLookaheadDoesNotAimThroughACorner();
    testLookaheadIsUnrestrictedOnAStraight();
    testLookaheadGrowsWithSpeed();
    testCruiseScaleIsFullOnAStraight();
    testCruiseScaleDropsApproachingACorner();
    testCruiseScaleRespectsItsFloor();
    testPathOffsetIsZeroOnTheRoute();
    testPathOffsetPointsBackToTheRoute();
    testReverseFliesTheRouteBackwards();
    testReverseKeepsRecordedHeadingsByDefault();
    testReverseHeadingsTurnsTheDroneAround();
    testRepeatKeepsFlyingUntilThePassesAreDone();
    testRepeatEventuallyCompletes();
    testPingPongAlternatesDirection();
    return test::summary("mission_planner");
}
