#include "common/frame_alignment.hpp"

#include <cmath>

namespace common {
namespace {

common::Vector3 cross(const common::Vector3& a, const common::Vector3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float dot(const common::Vector3& a, const common::Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

common::Vector3 scale(const common::Vector3& v, float s) { return {v.x * s, v.y * s, v.z * s}; }

common::Vector3 add(const common::Vector3& a, const common::Vector3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

common::Vector3 subtract(const common::Vector3& a, const common::Vector3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

common::Vector3 unit(const common::Vector3& v) {
    const float length = std::sqrt(dot(v, v));
    return length > 1e-6f ? scale(v, 1.0f / length) : common::Vector3{0.0f, 0.0f, 0.0f};
}

// Camera optical axes (x right, y down, z forward) expressed in body axes
// (x forward, y left, z up), for a camera bolted to the nose looking ahead:
//   camera +z (forward) -> body +x
//   camera +x (right)   -> body -y
//   camera +y (down)    -> body -z
common::Vector3 cameraToBodyAxes(const common::Vector3& c) {
    return {c.z, -c.x, -c.y};
}

common::Vector3 bodyToCameraAxes(const common::Vector3& b) {
    return {-b.y, -b.z, b.x};
}

// Rotates about the body's y axis (left), i.e. pitch, to undo a camera that
// is mounted looking downward by `degrees`.
common::Vector3 undoCameraPitch(const common::Vector3& body, float degrees) {
    if (degrees == 0.0f) return body;
    const float angle = degrees * static_cast<float>(M_PI) / 180.0f;
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    // Rotation about +y (left) by -angle: tilts the camera's forward axis
    // back up to the airframe's forward axis.
    return {c * body.x - s * body.z, body.y, s * body.x + c * body.z};
}

common::Vector3 applyCameraPitch(const common::Vector3& body, float degrees) {
    return undoCameraPitch(body, -degrees);
}

}  // namespace

common::Quaternion conjugate(const common::Quaternion& q) { return {q.w, -q.x, -q.y, -q.z}; }

common::Quaternion normalize(const common::Quaternion& q) {
    const float length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (length < 1e-9f) return {1.0f, 0.0f, 0.0f, 0.0f};
    return {q.w / length, q.x / length, q.y / length, q.z / length};
}

common::Vector3 rotate(const common::Quaternion& q, const common::Vector3& v) {
    // v' = v + 2 * qv x (qv x v + w * v), the standard branch-free form.
    const common::Vector3 qv{q.x, q.y, q.z};
    const common::Vector3 t = scale(cross(qv, v), 2.0f);
    return add(add(v, scale(t, q.w)), cross(qv, t));
}

float wrapAngle(float radians) {
    constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
    radians = std::fmod(radians + static_cast<float>(M_PI), kTwoPi);
    if (radians < 0.0f) radians += kTwoPi;
    return radians - static_cast<float>(M_PI);
}

float distance(const common::Vector3& a, const common::Vector3& b) {
    const common::Vector3 d = subtract(a, b);
    return std::sqrt(dot(d, d));
}

common::Vector3 FrameAlignment::mapToBody(const common::Quaternion& cameraInMap,
                                           const common::Vector3& mapVector) const {
    // Map -> camera is the inverse of the camera's orientation in the map.
    const common::Vector3 inCamera = rotate(conjugate(normalize(cameraInMap)), mapVector);
    return undoCameraPitch(cameraToBodyAxes(inCamera), camera_pitch_down_deg);
}

common::Vector3 FrameAlignment::bodyToMap(const common::Quaternion& cameraInMap,
                                           const common::Vector3& bodyVector) const {
    const common::Vector3 inCamera =
        bodyToCameraAxes(applyCameraPitch(bodyVector, camera_pitch_down_deg));
    return rotate(normalize(cameraInMap), inCamera);
}

float FrameAlignment::headingInMap(const common::Quaternion& cameraInMap) const {
    const common::Vector3 up = unit(map_up);

    // Where the airframe's nose points, in map coordinates.
    const common::Vector3 forward =
        rotate(normalize(cameraInMap), bodyToCameraAxes(
            applyCameraPitch({1.0f, 0.0f, 0.0f}, camera_pitch_down_deg)));

    // Build a horizontal basis perpendicular to `up`. The reference (zero
    // heading) direction is the map's +z projected onto that plane - the
    // direction the camera faced at the first keyframe.
    common::Vector3 reference = unit(subtract(common::Vector3{0.0f, 0.0f, 1.0f},
                                               scale(up, dot(common::Vector3{0.0f, 0.0f, 1.0f}, up))));
    if (distance(reference, {0.0f, 0.0f, 0.0f}) < 1e-6f) {
        // Degenerate: map_up is parallel to +z, so pick another axis.
        reference = unit(subtract(common::Vector3{1.0f, 0.0f, 0.0f},
                                   scale(up, dot(common::Vector3{1.0f, 0.0f, 0.0f}, up))));
    }
    const common::Vector3 left = cross(up, reference);

    const common::Vector3 flat = subtract(forward, scale(up, dot(forward, up)));
    return std::atan2(dot(flat, left), dot(flat, reference));
}

} // namespace common
