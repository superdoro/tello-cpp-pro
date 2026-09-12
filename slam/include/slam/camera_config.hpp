#pragma once

#include <string>

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
};

bool readCameraConfig(const std::string& path, CameraConfigInfo& info);

// Writes a config in the schema ORB-SLAM3 expects, preserving the explanatory
// comments that make config/tello_camera.yaml readable. Used by
// tools/calibrate_camera.
bool writeCameraConfig(const std::string& path, const CameraConfigInfo& info,
                        const double distortion[5]);

} // namespace slam
