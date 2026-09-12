#include <chrono>

#include "control/pid_flight_controller.hpp"
#include "test_check.hpp"

namespace {

common::PoseEstimate poseAt(float x, float y, float z) {
    common::PoseEstimate estimate;
    estimate.pose.position = {x, y, z};
    estimate.pose.orientation = {1.0f, 0.0f, 0.0f, 0.0f};  // facing map +z
    estimate.state = common::TrackingState::Ok;
    estimate.confidence = 1.0f;
    return estimate;
}

control::Waypoint waypointAt(float x, float y, float z) {
    control::Waypoint waypoint;
    waypoint.position = {x, y, z};
    return waypoint;
}

constexpr std::chrono::duration<double> kStep{0.05};

void testDrivesForwardTowardsATargetAhead() {
    test::beginCase("a target straight ahead commands forward pitch only");
    control::PidFlightController controller;

    const auto command = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1), kStep);
    CHECK(command.pitch > 0);
    CHECK_EQ(command.roll, 0);
    CHECK_EQ(command.throttle, 0);
    CHECK_EQ(command.yaw, 0);
}

void testTargetToTheRightRollsRight() {
    test::beginCase("a target to the right commands positive roll");
    control::PidFlightController controller;

    // Map +x is to the camera's right when facing +z.
    const auto command = controller.computeCommand(poseAt(0, 0, 0), waypointAt(1, 0, 0), kStep);
    CHECK(command.roll > 0);
    CHECK_EQ(command.pitch, 0);
}

void testTargetAboveCommandsPositiveThrottle() {
    test::beginCase("a target above commands positive throttle");
    control::PidFlightController controller;

    // Map -y is up under the default alignment.
    const auto command = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, -1, 0), kStep);
    CHECK(command.throttle > 0);
}

void testCommandsAreClamped() {
    test::beginCase("a huge error is still clamped to the configured ceiling");
    control::FlightControllerConfig config;
    config.max_horizontal_command = 20;
    config.max_vertical_command = 15;
    control::PidFlightController controller(config);

    const auto command =
        controller.computeCommand(poseAt(0, 0, 0), waypointAt(100, -100, 100), kStep);
    CHECK(std::abs(command.pitch) <= 20);
    CHECK(std::abs(command.roll) <= 20);
    CHECK(std::abs(command.throttle) <= 15);
}

void testUntrackedPoseCommandsHover() {
    test::beginCase("an untracked pose commands a hover, whatever the target");
    control::PidFlightController controller;

    common::PoseEstimate lost = poseAt(0, 0, 0);
    lost.state = common::TrackingState::Lost;

    const auto command = controller.computeCommand(lost, waypointAt(0, 0, 5), kStep);
    CHECK_EQ(command.roll, 0);
    CHECK_EQ(command.pitch, 0);
    CHECK_EQ(command.throttle, 0);
    CHECK_EQ(command.yaw, 0);
}

void testDeadbandSettles() {
    test::beginCase("an error inside the deadband commands nothing");
    control::FlightControllerConfig config;
    config.position_deadband_m = 0.1f;
    control::PidFlightController controller(config);

    const auto command = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.05f), kStep);
    CHECK_EQ(command.pitch, 0);
}

void testResetClearsIntegral() {
    test::beginCase("reset() clears the integrator so it cannot wind up while blind");
    control::PidFlightController controller;

    // A long-standing offset builds up integral action.
    for (int i = 0; i < 40; ++i) {
        controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.5f), kStep);
    }
    const auto wound = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.5f), kStep);

    controller.reset();
    const auto fresh = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.5f), kStep);

    CHECK(fresh.pitch <= wound.pitch);
}

void testYawOnlyWhenHeadingIsHeld() {
    test::beginCase("yaw is commanded only for waypoints that hold a heading");
    control::PidFlightController controller;

    control::Waypoint free = waypointAt(0, 0, 1);
    free.hold_heading = false;
    CHECK_EQ(controller.computeCommand(poseAt(0, 0, 0), free, kStep).yaw, 0);

    controller.reset();
    control::Waypoint held = waypointAt(0, 0, 1);
    held.hold_heading = true;
    held.heading_rad = 1.0f;  // 57 degrees off the drone's current heading of 0
    CHECK(controller.computeCommand(poseAt(0, 0, 0), held, kStep).yaw != 0);
}

// Both of these reproduce spikes that showed up as one-frame full-deflection
// stick commands in an end-to-end dry run - the kind of thing that reads as a
// violent twitch on a real airframe.
void testNoSpikeWhenTheWaypointChanges() {
    test::beginCase("switching waypoint does not kick the sticks");
    control::PidFlightController controller;

    // Settle on a target close ahead.
    common::VelocityCommand settled;
    for (int i = 0; i < 30; ++i) {
        settled = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.06f), kStep);
    }

    // The mission moves on to a waypoint 2m away: the error jumps by 2m in a
    // single frame, which a naive derivative reads as 40 m/s of closing speed.
    const auto afterSwitch =
        controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 2.0f), kStep);

    // Proportional action alone on a 2m error already saturates, so the test
    // is that the command is not driven the WRONG way or beyond the clamp.
    CHECK(afterSwitch.pitch > 0);
    CHECK(afterSwitch.pitch <= controller.config().max_horizontal_command);
    CHECK_EQ(afterSwitch.roll, 0);
    CHECK_EQ(afterSwitch.throttle, 0);
}

void testDeadbandIsContinuous() {
    test::beginCase("commands stay continuous across the deadband boundary");
    control::FlightControllerConfig config;
    config.position_deadband_m = 0.05f;
    config.forward.kd = 0.0f;  // isolate the deadband shape itself
    config.forward.ki = 0.0f;
    control::PidFlightController controller(config);

    // Just inside and just outside the boundary must differ by almost nothing.
    control::PidFlightController a(config);
    control::PidFlightController b(config);
    const auto inside = a.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.049f), kStep);
    const auto outside = b.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.051f), kStep);
    CHECK(std::abs(outside.pitch - inside.pitch) <= 1);

    // And a hard deadband would make this one big: at the boundary the error
    // would snap from 0.05 to 0, which is what produced the observed spikes.
    control::PidFlightController c(config);
    const auto wellOutside = c.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.15f), kStep);
    CHECK(wellOutside.pitch > outside.pitch);
}

void testDerivativeIsFiltered() {
    test::beginCase("a one-frame position glitch does not reach the sticks in full");
    control::FlightControllerConfig config;
    config.forward.kp = 0.0f;  // derivative only
    config.forward.ki = 0.0f;
    config.forward.kd = 50.0f;
    config.derivative_filter_s = 0.2f;
    control::PidFlightController controller(config);

    const auto target = waypointAt(0, 0, 1.0f);
    for (int i = 0; i < 10; ++i) controller.computeCommand(poseAt(0, 0, 0), target, kStep);

    // One frame of SLAM jitter: 10cm of apparent motion in a single step.
    const auto glitched = controller.computeCommand(poseAt(0, 0, 0.1f), target, kStep);

    // Unfiltered this would be 0.1/0.05 * 50 = 100 stick units. The filter
    // must keep it far below the clamp for a single sample.
    CHECK(std::abs(glitched.pitch) < config.max_horizontal_command);
}

void testCruiseRaisesForwardCommandOnOpenRoute() {
    test::beginCase("the cruise term lifts the forward stick beyond kp x error");
    control::FlightControllerConfig config;
    config.max_horizontal_command = 100;

    control::PidFlightController plain(config);
    const int withoutCruise =
        plain.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).pitch;

    config.cruise_command = 50;
    control::PidFlightController cruising(config);
    const int withCruise =
        cruising.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).pitch;

    CHECK(withCruise > withoutCruise);
    CHECK(withCruise <= 100);
}

void testCruiseFadesOutAtTheTarget() {
    test::beginCase("cruise fades to nothing as the target is reached, so it still stops");
    control::FlightControllerConfig config;
    config.max_horizontal_command = 100;
    config.cruise_command = 50;
    config.cruise_fade_m = 1.0f;
    control::PidFlightController controller(config);

    const int far = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).pitch;
    controller.reset();
    const int close =
        controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 0.1f), kStep).pitch;

    CHECK(close < far / 2);
}

void testCruiseNeverPushesTowardsATargetBehind() {
    test::beginCase("cruise does not push forward when the target is behind");
    control::FlightControllerConfig config;
    config.max_horizontal_command = 100;
    config.cruise_command = 60;
    control::PidFlightController controller(config);

    // Facing +z, target 1m behind at -z.
    const auto command =
        controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, -1.0f), kStep);
    CHECK(command.pitch < 0);
}

void testCenteringStrengthensTheReturnToTheRoute() {
    test::beginCase("the centering term adds lateral correction towards the route");
    control::FlightControllerConfig config;
    config.max_horizontal_command = 100;

    // Facing map +z, target straight ahead, but sitting off to the side: the
    // route is 0.5m to the drone's left (map -x, which is body +y).
    const common::Vector3 offsetTowardsRoute{-0.5f, 0.0f, 0.0f};

    config.path_centering = 0.0f;
    control::PidFlightController plain(config);
    plain.setPathOffset(offsetTowardsRoute);
    const int withoutCentering =
        plain.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).roll;

    config.path_centering = 1.0f;
    control::PidFlightController centred(config);
    centred.setPathOffset(offsetTowardsRoute);
    const int withCentering =
        centred.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).roll;

    // `rc` roll is positive to the right; the route is to the left.
    CHECK(withCentering < withoutCentering);
    CHECK(withCentering < 0);
}

void testCenteringDoesNothingOnTheRoute() {
    test::beginCase("no centering command when already on the route");
    control::FlightControllerConfig config;
    config.path_centering = 1.0f;
    control::PidFlightController controller(config);
    controller.setPathOffset({0.0f, 0.0f, 0.0f});

    const auto command = controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep);
    CHECK_EQ(command.roll, 0);
}

void testCenteringDoesNotChangeForwardSpeed() {
    test::beginCase("centering never touches the along-path channel");
    control::FlightControllerConfig config;
    config.max_horizontal_command = 100;
    config.path_centering = 1.0f;

    control::PidFlightController controller(config);
    controller.setPathOffset({0.0f, 0.0f, 0.0f});
    const int onRoute =
        controller.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).pitch;

    control::PidFlightController offset(config);
    // A purely sideways offset must not slow the drone down or speed it up.
    offset.setPathOffset({-0.5f, 0.0f, 0.0f});
    const int offRoute =
        offset.computeCommand(poseAt(0, 0, 0), waypointAt(0, 0, 1.5f), kStep).pitch;

    CHECK_EQ(onRoute, offRoute);
}

}  // namespace

int main() {
    std::cout << "pid_flight_controller\n";
    testDrivesForwardTowardsATargetAhead();
    testTargetToTheRightRollsRight();
    testTargetAboveCommandsPositiveThrottle();
    testCommandsAreClamped();
    testUntrackedPoseCommandsHover();
    testDeadbandSettles();
    testResetClearsIntegral();
    testYawOnlyWhenHeadingIsHeld();
    testNoSpikeWhenTheWaypointChanges();
    testDeadbandIsContinuous();
    testDerivativeIsFiltered();
    testCruiseRaisesForwardCommandOnOpenRoute();
    testCruiseFadesOutAtTheTarget();
    testCruiseNeverPushesTowardsATargetBehind();
    testCenteringStrengthensTheReturnToTheRoute();
    testCenteringDoesNothingOnTheRoute();
    testCenteringDoesNotChangeForwardSpeed();
    return test::summary("pid_flight_controller");
}
