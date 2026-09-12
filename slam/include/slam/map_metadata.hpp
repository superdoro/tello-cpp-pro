#pragma once

#include <string>

namespace slam {

// Sidecar written next to every map (`<map>.osa` -> `<map>.meta.yaml`).
//
// It exists because a monocular ORB-SLAM3 map is not self-describing in the
// two ways that matter for flying against it:
//
//  1. Scale. Monocular SLAM reconstructs geometry only up to an unknown
//     scale factor, so map coordinates are in arbitrary units. Without the
//     measured metres-per-unit, "fly 2 units forward" means nothing.
//  2. Provenance. A map only relocalizes against images from the same camera
//     at the same resolution, processed with the same vocabulary. Recording
//     which files produced it turns a silent tracking failure into a clear
//     mismatch error.
struct MapMetadata {
    int version = 1;

    // Metres per SLAM unit. 1.0 means "unknown / uncalibrated", which is what
    // a freshly built map has until tools/calibrate_scale measures it.
    double scale_metres_per_unit = 1.0;
    bool scale_calibrated = false;

    std::string camera_config;    // path to the intrinsics used
    std::string vocabulary;       // path to the ORB vocabulary used
    std::string source;           // video file or "live:<drone ip>"
    std::string created_utc;
    std::string trajectory_file;  // TUM keyframe trajectory, if written

    int image_width = 0;
    int image_height = 0;
    // Keyframes the map ended up with. A map with very few is a map that will
    // relocalize badly.
    int keyframes = 0;
    unsigned long long frames_processed = 0;
    unsigned long long frames_tracked = 0;
};

// Both return false (and log) rather than throwing, so a missing or
// hand-edited sidecar degrades to "scale unknown" instead of killing a flight.
bool saveMapMetadata(const std::string& mapPath, const MapMetadata& metadata);
bool loadMapMetadata(const std::string& mapPath, MapMetadata& metadata);

// `<map>.osa` -> `<map>.meta.yaml`
std::string metadataPathForMap(const std::string& mapPath);

} // namespace slam
