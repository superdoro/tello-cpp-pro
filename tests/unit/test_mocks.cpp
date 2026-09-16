#include <opencv2/core.hpp>

#include "mocks/mock_drone.hpp"
#include "mocks/mock_frame_source.hpp"
#include "mocks/mock_localizer.hpp"
#include "slam/camera_config.hpp"
#include "test_check.hpp"

namespace {

common::CameraIntrinsics telloIntrinsics() {
    // config/tello_camera.yaml, so the mocks are exercised against the real
    // numbers rather than round ones that hide scaling mistakes.
    common::CameraIntrinsics k;
    k.width = 960;
    k.height = 720;
    k.fx = 923.669841f;
    k.fy = 925.404802f;
    k.cx = 458.722537f;
    k.cy = 352.460091f;
    k.k1 = -0.017206f;
    k.k2 = -0.068845f;
    k.p1 = -0.008177f;
    k.p2 = -0.002914f;
    k.k3 = 0.232057f;
    return k;
}

void testMockDroneRecordsEveryCommand() {
    test::beginCase("MockDrone records the command stream a real flight would get");
    mocks::MockDrone drone;
    CHECK(drone.connect());
    CHECK(drone.takeoff());
    CHECK(drone.airborne);

    drone.sendVelocity({0, 0, 0, 0});
    drone.sendVelocity({0, 20, 0, 0});
    drone.sendVelocity({-5, 12, 3, -8});

    CHECK_EQ(drone.commands.size(), 3u);
    CHECK_EQ(drone.last_command.pitch, 12);
    CHECK_EQ(drone.maxAbsPitch(), 20);
    CHECK(!drone.onlyEverHovered());

    CHECK(drone.land());
    CHECK(!drone.airborne);
    CHECK_EQ(drone.land_calls, 1);
}

void testMockDroneCanFailOnDemand() {
    test::beginCase("MockDrone can refuse to take off");
    mocks::MockDrone drone;
    drone.takeoff_succeeds = false;
    CHECK(!drone.takeoff());
    CHECK(!drone.airborne);
    CHECK_EQ(drone.takeoff_calls, 1);
}

void testMockFrameSourcePlaysBackAndStalls() {
    test::beginCase("MockFrameSource replays frames and can go silent");
    mocks::MockFrameSource source;
    source.appendFrame(cv::Mat::zeros(4, 4, CV_8UC3), 0.0);
    source.appendFrame(cv::Mat::zeros(4, 4, CV_8UC3), 1.0 / 30.0);
    CHECK(source.open());

    const auto first = source.nextFrame(100);
    CHECK(first.has_value());
    CHECK_EQ(first->index, 0u);
    CHECK_EQ(source.last_timeout_ms, 100);

    // A stalled live feed: still open, but nothing arrives. This is the case
    // the mission app's 2-second video watchdog exists for.
    source.stalled = true;
    CHECK(!source.nextFrame(100).has_value());
    CHECK(source.isOpen());

    source.stalled = false;
    CHECK(source.nextFrame(100).has_value());

    // End of a file source: closes, so isOpen() distinguishes it from a stall.
    CHECK(!source.nextFrame(100).has_value());
    CHECK(!source.isOpen());
}

void testMockLocalizerScriptsTrackingLoss() {
    test::beginCase("MockLocalizer scripts a tracking dropout");
    mocks::MockLocalizer localizer;
    localizer.appendTracked({0.0f, 0.0f, 0.0f});
    localizer.appendLost(2);
    localizer.appendTracked({0.0f, 0.0f, 1.0f});

    video::Frame frame;
    CHECK(localizer.processFrame(frame).state == common::TrackingState::Ok);
    CHECK(localizer.processFrame(frame).state == common::TrackingState::Lost);
    CHECK(localizer.processFrame(frame).state == common::TrackingState::Lost);
    const auto recovered = localizer.processFrame(frame);
    CHECK(recovered.state == common::TrackingState::Ok);
    CHECK_NEAR(recovered.pose.position.z, 1.0, 1e-6);
}

void testMockLocalizerDistinguishesUnmatchedFromTracked() {
    test::beginCase("MockLocalizer models tracking a throwaway map");
    // Ok tracking in a map whose waypoints mean nothing: in_prebuilt_map, not
    // TrackingState, is what a mission start must gate on.
    mocks::MockLocalizer localizer;
    localizer.appendUnmatched(1);
    localizer.appendTracked({1.0f, 0.0f, 0.0f});

    video::Frame frame;
    localizer.processFrame(frame);
    CHECK(!localizer.stats().in_prebuilt_map);
    localizer.processFrame(frame);
    CHECK(localizer.stats().in_prebuilt_map);
}

void testMockLocalizerScriptsAMapFrameJump() {
    test::beginCase("MockLocalizer scripts a loop closure shifting the map frame");
    mocks::MockLocalizer localizer;
    localizer.appendTracked({0.0f, 0.0f, 0.0f});
    const int keyframesBefore = localizer.current().stats.map_keyframes;
    localizer.appendMapFrameJump({2.5f, 0.0f, -1.0f});

    video::Frame frame;
    localizer.processFrame(frame);
    const auto after = localizer.processFrame(frame);

    CHECK_NEAR(after.pose.position.x, 2.5, 1e-6);
    CHECK_NEAR(after.pose.position.z, -1.0, 1e-6);
    // Both signals a consumer can detect the shift by.
    CHECK(localizer.stats().map_keyframes > keyframesBefore);
    CHECK(localizer.stats().map_id != 0);
}

void testMockLocalizerSuppliesSparseDepth() {
    test::beginCase("MockLocalizer supplies synthetic sparse metric depth");
    mocks::MockLocalizer localizer;
    common::SparseDepthFrame depth;

    // Nothing attached: must report failure, not empty success.
    localizer.appendTracked({0.0f, 0.0f, 0.0f});
    CHECK(!localizer.trackedDepthSamples(depth));

    const auto intrinsics = telloIntrinsics();
    localizer.appendTracked({0.0f, 0.0f, 0.5f});
    localizer.attachPlanarDepth(intrinsics, 2.5f, 64);

    video::Frame frame;
    localizer.processFrame(frame);
    localizer.processFrame(frame);
    CHECK(localizer.trackedDepthSamples(depth));
    CHECK_EQ(depth.samples.size(), 64u);
    CHECK_EQ(depth.image_width, 960);

    for (const auto& sample : depth.samples) {
        CHECK_NEAR(sample.depth_m, 2.5, 1e-5);
        CHECK(sample.u >= 0.0f && sample.u <= 960.0f);
        CHECK(sample.v >= 0.0f && sample.v <= 720.0f);
    }
}

void testToIntrinsicsCarriesDistortion() {
    test::beginCase("slam::toIntrinsics carries the distortion coefficients across");
    slam::CameraConfigInfo info;
    info.width = 960;
    info.height = 720;
    info.fx = 923.669841;
    info.fy = 925.404802;
    info.cx = 458.722537;
    info.cy = 352.460091;
    info.k1 = -0.017206;
    info.k3 = 0.232057;

    const common::CameraIntrinsics k = slam::toIntrinsics(info);
    CHECK(k.valid());
    CHECK(k.hasDistortion());
    CHECK_NEAR(k.fx, 923.669841, 1e-3);
    CHECK_NEAR(k.k1, -0.017206, 1e-6);
    CHECK_NEAR(k.k3, 0.232057, 1e-6);

    // 55 degrees horizontal. This is the hard limit on how far to the side an
    // avoidance manoeuvre may aim, so it is worth asserting rather than
    // recomputing by hand every time someone changes a lens.
    CHECK_NEAR(common::horizontalFovRad(k) * 180.0 / M_PI, 54.9, 0.5);
    CHECK_NEAR(common::verticalFovRad(k) * 180.0 / M_PI, 42.7, 0.5);
}

}  // namespace

int main() {
    std::cout << "mocks\n";
    testMockDroneRecordsEveryCommand();
    testMockDroneCanFailOnDemand();
    testMockFrameSourcePlaysBackAndStalls();
    testMockLocalizerScriptsTrackingLoss();
    testMockLocalizerDistinguishesUnmatchedFromTracked();
    testMockLocalizerScriptsAMapFrameJump();
    testMockLocalizerSuppliesSparseDepth();
    testToIntrinsicsCarriesDistortion();
    return test::summary("mocks");
}
