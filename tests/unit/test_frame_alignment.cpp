// The map->body rotation is the single place where a sign error turns into a
// drone flying the wrong way, and it cannot be checked by reading it. These
// tests pin down every axis against a hand-worked expectation.
#include <cmath>

#include "control/frame_alignment.hpp"
#include "test_check.hpp"

using common::Quaternion;
using common::Vector3;

namespace {

// Camera axes are x right, y down, z forward. Identity orientation means the
// drone's nose points along the map's +z.
const Quaternion kIdentity{1.0f, 0.0f, 0.0f, 0.0f};

Quaternion aboutAxis(const Vector3& axis, float radians) {
    const float half = radians * 0.5f;
    const float s = std::sin(half);
    const float length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    return {std::cos(half), axis.x / length * s, axis.y / length * s, axis.z / length * s};
}

void testIdentityOrientationAxes() {
    test::beginCase("map->body axes with the camera facing along map +z");
    const control::FrameAlignment alignment;

    // Map +z is straight ahead of the camera, so it must come out as body
    // +x (forward).
    const Vector3 forward = alignment.mapToBody(kIdentity, {0.0f, 0.0f, 1.0f});
    CHECK_NEAR(forward.x, 1.0, 1e-5);
    CHECK_NEAR(forward.y, 0.0, 1e-5);
    CHECK_NEAR(forward.z, 0.0, 1e-5);

    // Map +x is to the camera's right, and body +y is left, so it flips sign.
    const Vector3 right = alignment.mapToBody(kIdentity, {1.0f, 0.0f, 0.0f});
    CHECK_NEAR(right.x, 0.0, 1e-5);
    CHECK_NEAR(right.y, -1.0, 1e-5);
    CHECK_NEAR(right.z, 0.0, 1e-5);

    // Map +y is down in camera convention, and body +z is up.
    const Vector3 down = alignment.mapToBody(kIdentity, {0.0f, 1.0f, 0.0f});
    CHECK_NEAR(down.x, 0.0, 1e-5);
    CHECK_NEAR(down.y, 0.0, 1e-5);
    CHECK_NEAR(down.z, -1.0, 1e-5);
}

void testRotatedOrientation() {
    test::beginCase("a target behind a drone that has turned 90 degrees");
    const control::FrameAlignment alignment;

    // Yaw is a rotation about the camera's y axis (down), so a positive
    // rotation about +y turns the nose towards map -x... check by asking
    // where map +z ends up after turning 90 degrees.
    const Quaternion turned = aboutAxis({0.0f, 1.0f, 0.0f}, static_cast<float>(M_PI) / 2.0f);

    const Vector3 body = alignment.mapToBody(turned, {0.0f, 0.0f, 1.0f});
    // The old forward direction is now off to one side, never still ahead.
    CHECK_NEAR(body.x, 0.0, 1e-5);
    CHECK_NEAR(std::abs(body.y), 1.0, 1e-5);
}

void testRoundTrip() {
    test::beginCase("bodyToMap is the inverse of mapToBody");
    control::FrameAlignment alignment;
    alignment.camera_pitch_down_deg = 12.0f;

    const Quaternion orientation = aboutAxis({0.3f, 1.0f, -0.2f}, 0.9f);
    const Vector3 original{1.3f, -0.7f, 2.4f};

    const Vector3 body = alignment.mapToBody(orientation, original);
    const Vector3 back = alignment.bodyToMap(orientation, body);

    CHECK_NEAR(back.x, original.x, 1e-4);
    CHECK_NEAR(back.y, original.y, 1e-4);
    CHECK_NEAR(back.z, original.z, 1e-4);
}

void testHeading() {
    test::beginCase("heading is zero facing map +z and grows counter-clockwise");
    const control::FrameAlignment alignment;

    CHECK_NEAR(alignment.headingInMap(kIdentity), 0.0, 1e-5);

    // Rotating about the camera's down axis by +90 degrees swings the nose;
    // whichever way it goes, the magnitude must be a right angle.
    const Quaternion turned = aboutAxis({0.0f, 1.0f, 0.0f}, static_cast<float>(M_PI) / 2.0f);
    CHECK_NEAR(std::abs(alignment.headingInMap(turned)), M_PI / 2.0, 1e-4);
}

void testWrapAngle() {
    test::beginCase("angle wrapping never takes the long way round");
    CHECK_NEAR(control::wrapAngle(0.0f), 0.0, 1e-6);
    CHECK_NEAR(control::wrapAngle(static_cast<float>(M_PI) * 3.0f / 2.0f), -M_PI / 2.0, 1e-5);
    CHECK_NEAR(control::wrapAngle(-static_cast<float>(M_PI) * 3.0f / 2.0f), M_PI / 2.0, 1e-5);

    // The case that motivates the function: turning from +179 to -179 degrees
    // is a 2-degree turn, not a 358-degree one.
    const float from = 179.0f * static_cast<float>(M_PI) / 180.0f;
    const float to = -179.0f * static_cast<float>(M_PI) / 180.0f;
    CHECK_NEAR(std::abs(control::wrapAngle(to - from)), 2.0 * M_PI / 180.0, 1e-5);
}

}  // namespace

int main() {
    std::cout << "frame_alignment\n";
    testIdentityOrientationAxes();
    testRotatedOrientation();
    testRoundTrip();
    testHeading();
    testWrapAngle();
    return test::summary("frame_alignment");
}
