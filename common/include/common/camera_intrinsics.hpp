#pragma once

#include <cmath>

namespace common {

// Pinhole intrinsics plus Brown-Conrady distortion, in the resolution the
// calibration was measured at.
//
// This duplicates part of slam::CameraConfigInfo on purpose. perception/ needs
// intrinsics to turn a depth map into a point cloud, and making it include a
// slam/ header to get them would couple two modules that must stay siblings.
// slam::toIntrinsics() converts, so there is still exactly one file on disk
// these numbers come from.
struct CameraIntrinsics {
    int width = 0;
    int height = 0;

    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;

    // Radial (k) and tangential (p) distortion, OpenCV's ordering.
    float k1 = 0.0f;
    float k2 = 0.0f;
    float p1 = 0.0f;
    float p2 = 0.0f;
    float k3 = 0.0f;

    bool valid() const { return width > 0 && height > 0 && fx > 0.0f && fy > 0.0f; }

    // Whether a distortion model is present at all.
    //
    // Worth checking rather than assuming: ORB-SLAM3's Frame::UndistortKeyPoints()
    // short-circuits and leaves mvKeysUn == mvKeys when k1 is exactly zero, so
    // "are these keypoints undistorted?" and "does this camera have distortion?"
    // are the same question, and a synthetic zero-distortion camera answers no.
    bool hasDistortion() const {
        return k1 != 0.0f || k2 != 0.0f || p1 != 0.0f || p2 != 0.0f || k3 != 0.0f;
    }
};

// A point on the image plane, in pixels.
struct Pixel {
    float u = 0.0f;
    float v = 0.0f;
};

// A direction out of the camera, normalised so z == 1. Multiply by a metric
// depth to get a 3D point in the CAMERA frame (x right, y down, z forward).
struct CameraRay {
    float x = 0.0f;
    float y = 0.0f;
};

// Ideal (distortion-free) pixel -> normalised ray. The plain pinhole inverse.
inline CameraRay rayFromIdealPixel(const Pixel& p, const CameraIntrinsics& k) {
    return {(p.u - k.cx) / k.fx, (p.v - k.cy) / k.fy};
}

// Normalised ray -> ideal pixel.
inline Pixel projectIdeal(const CameraRay& r, const CameraIntrinsics& k) {
    return {r.x * k.fx + k.cx, r.y * k.fy + k.cy};
}

// Ideal pixel -> the RAW pixel the lens actually puts that ray on.
//
// This is the forward Brown-Conrady model, which is closed-form; the inverse
// is not, which is why undistortion is left to perception/ where OpenCV's
// iterative cv::undistortPoints is available.
//
// Needed because the only keypoints ORB-SLAM3 exposes are undistorted
// (GetTrackedKeyPointsUn; mvKeys is behind a private member), while a depth
// network sees the raw image. Sampling the depth map at undistorted
// coordinates is wrong by ~8 px at the corners of this project's camera -
// enough to read the far side of an object boundary.
inline Pixel distortPixel(const Pixel& ideal, const CameraIntrinsics& k) {
    if (!k.hasDistortion()) return ideal;

    const CameraRay r = rayFromIdealPixel(ideal, k);
    const float x = r.x;
    const float y = r.y;
    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;

    const float radial = 1.0f + k.k1 * r2 + k.k2 * r4 + k.k3 * r6;
    const float xd = x * radial + 2.0f * k.p1 * x * y + k.p2 * (r2 + 2.0f * x * x);
    const float yd = y * radial + k.p1 * (r2 + 2.0f * y * y) + 2.0f * k.p2 * x * y;

    return {xd * k.fx + k.cx, yd * k.fy + k.cy};
}

// Horizontal / vertical field of view in radians. Worth having in one place:
// this camera's 55 degrees horizontal is the hard limit on how far to the side
// an obstacle-avoidance manoeuvre may aim, because beyond it there is no
// current evidence at all.
inline float horizontalFovRad(const CameraIntrinsics& k) {
    return k.valid() ? 2.0f * std::atan(static_cast<float>(k.width) * 0.5f / k.fx) : 0.0f;
}

inline float verticalFovRad(const CameraIntrinsics& k) {
    return k.valid() ? 2.0f * std::atan(static_cast<float>(k.height) * 0.5f / k.fy) : 0.0f;
}

} // namespace common
