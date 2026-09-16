#pragma once

#include <string>

#include "common/camera_intrinsics.hpp"

namespace slam {

// The handful of fields other modules need out of the ORB-SLAM3 camera YAML.
// Reading them here keeps apps from parsing the same file three different
// ways - and, more importantly, lets every video source be told the exact
// resolution the intrinsics were measured at.
struct CameraConfigInfo {
    int width = 0;
    int height = 0;
    int fps = 30;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    // Brown-Conrady distortion, in OpenCV's k1 k2 p1 p2 k3 order.
    //
    // Carried in the struct rather than passed alongside it, so there is one
    // answer to "what distortion does this camera have" rather than two that
    // can disagree.
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
};

bool readCameraConfig(const std::string& path, CameraConfigInfo& info);

// Writes a config in the schema ORB-SLAM3 expects, preserving the explanatory
// comments that make config/tello_camera.yaml readable. Used by
// tools/calibrate_camera.
bool writeCameraConfig(const std::string& path, const CameraConfigInfo& info);

// Narrows to the module-neutral form in common/, so perception/ can have the
// intrinsics without including a slam/ header.
//
// Inline on purpose: this must be callable by code that only sees slam's
// headers (tests, mocks) without linking the ORB-SLAM3 build.
inline common::CameraIntrinsics toIntrinsics(const CameraConfigInfo& info) {
    common::CameraIntrinsics k;
    k.width = info.width;
    k.height = info.height;
    k.fx = static_cast<float>(info.fx);
    k.fy = static_cast<float>(info.fy);
    k.cx = static_cast<float>(info.cx);
    k.cy = static_cast<float>(info.cy);
    k.k1 = static_cast<float>(info.k1);
    k.k2 = static_cast<float>(info.k2);
    k.p1 = static_cast<float>(info.p1);
    k.p2 = static_cast<float>(info.p2);
    k.k3 = static_cast<float>(info.k3);
    return k;
}

} // namespace slam
