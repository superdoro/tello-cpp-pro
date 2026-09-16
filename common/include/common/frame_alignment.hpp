#pragma once

#include "common/types.hpp"

namespace common {

// Converting a SLAM pose into a drone command means crossing three frames,
// and getting any of them wrong produces a drone that confidently flies the
// wrong way. They are, explicitly:
//
//  1. CAMERA frame (what ORB-SLAM3 speaks): x right, y down, z forward.
//  2. MAP frame: simply the camera frame of the first keyframe, frozen. So
//     the map's axes are also "x right, y down, z forward", oriented however
//     the camera happened to point when mapping began - NOT any world frame.
//  3. BODY frame (what `rc a b c d` speaks): x forward, y left, z up.
//
// The useful consequence of (1) and (2) sharing a convention: the rotation
// part of the SLAM pose already tells us where the drone is pointing inside
// the map, so no separate "align the map to the body at takeoff" calibration
// flight is needed. A displacement in the map is rotated into the current
// camera frame by the inverse of the current orientation, then into the body
// frame by the fixed camera-mount rotation below.
//
// What SLAM still cannot tell us, and what must be configured, is which
// direction is up: the map frame's "up" is whatever -y was at the first
// keyframe. If the drone was level when mapping started (it is, if mapping
// starts from a hover), map -y is world up, which is the default here.
struct FrameAlignment {
    // Unit vector, in MAP coordinates, pointing up (against gravity).
    // Default = -y, correct when the camera was level at the first keyframe.
    common::Vector3 map_up{0.0f, -1.0f, 0.0f};

    // Downward tilt of the camera relative to the airframe, in degrees.
    // Positive means the camera looks below the horizon when the drone is
    // level. Non-zero for a Tello whose camera is not perfectly forward.
    float camera_pitch_down_deg = 0.0f;

    // Rotates a displacement expressed in the MAP frame into the BODY frame,
    // given the drone camera's current orientation in the map (the rotation
    // part of the pose ILocalizer returns).
    common::Vector3 mapToBody(const common::Quaternion& cameraInMap,
                               const common::Vector3& mapVector) const;

    // Inverse of mapToBody, for turning a body-frame velocity back into map
    // coordinates (used by tests and by dead-reckoning diagnostics).
    common::Vector3 bodyToMap(const common::Quaternion& cameraInMap,
                               const common::Vector3& bodyVector) const;

    // The drone's heading within the map's horizontal plane, in radians,
    // measured about map_up. The zero direction is the map frame's +z (i.e.
    // whichever way the camera pointed at the first keyframe), and positive
    // is counter-clockwise seen from above.
    float headingInMap(const common::Quaternion& cameraInMap) const;
};

// Rotates `v` by quaternion `q`. Exposed because both the alignment code and
// its tests need it, and it is easy to get subtly wrong.
common::Vector3 rotate(const common::Quaternion& q, const common::Vector3& v);

common::Quaternion conjugate(const common::Quaternion& q);
common::Quaternion normalize(const common::Quaternion& q);

// Wraps an angle to [-pi, pi]. Every yaw error must pass through this, or
// turning from +179 to -179 degrees becomes a 358-degree spin.
float wrapAngle(float radians);

float distance(const common::Vector3& a, const common::Vector3& b);

} // namespace common
