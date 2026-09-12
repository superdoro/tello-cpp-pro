#include "slam/map_metadata.hpp"

#include <filesystem>

#include <opencv2/core/persistence.hpp>

#include "common/logging.hpp"

namespace slam {

std::string metadataPathForMap(const std::string& mapPath) {
    std::filesystem::path path(mapPath);
    path.replace_extension();
    return path.string() + ".meta.yaml";
}

bool saveMapMetadata(const std::string& mapPath, const MapMetadata& metadata) {
    const std::string path = metadataPathForMap(mapPath);
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        common::logError("MapMetadata", "cannot write metadata to " + path);
        return false;
    }

    fs << "version" << metadata.version;
    fs << "scale_metres_per_unit" << metadata.scale_metres_per_unit;
    fs << "scale_calibrated" << (metadata.scale_calibrated ? 1 : 0);
    fs << "camera_config" << metadata.camera_config;
    fs << "vocabulary" << metadata.vocabulary;
    fs << "source" << metadata.source;
    fs << "created_utc" << metadata.created_utc;
    fs << "trajectory_file" << metadata.trajectory_file;
    fs << "image_width" << metadata.image_width;
    fs << "image_height" << metadata.image_height;
    fs << "keyframes" << metadata.keyframes;
    fs << "frames_processed" << static_cast<int>(metadata.frames_processed);
    fs << "frames_tracked" << static_cast<int>(metadata.frames_tracked);
    fs.release();

    common::logInfo("MapMetadata", "wrote " + path);
    return true;
}

bool loadMapMetadata(const std::string& mapPath, MapMetadata& metadata) {
    const std::string path = metadataPathForMap(mapPath);
    if (!std::filesystem::exists(path)) {
        common::logWarn("MapMetadata",
                         "no metadata sidecar at " + path + " - treating scale as uncalibrated");
        return false;
    }

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        common::logError("MapMetadata", "cannot read metadata at " + path);
        return false;
    }

    const auto readInt = [&fs](const char* key, int fallback) {
        const cv::FileNode node = fs[key];
        return node.empty() ? fallback : static_cast<int>(node);
    };
    const auto readString = [&fs](const char* key) {
        const cv::FileNode node = fs[key];
        return node.empty() ? std::string{} : static_cast<std::string>(node);
    };

    metadata.version = readInt("version", 1);
    const cv::FileNode scaleNode = fs["scale_metres_per_unit"];
    metadata.scale_metres_per_unit = scaleNode.empty() ? 1.0 : static_cast<double>(scaleNode);
    metadata.scale_calibrated = readInt("scale_calibrated", 0) != 0;
    metadata.camera_config = readString("camera_config");
    metadata.vocabulary = readString("vocabulary");
    metadata.source = readString("source");
    metadata.created_utc = readString("created_utc");
    metadata.trajectory_file = readString("trajectory_file");
    metadata.image_width = readInt("image_width", 0);
    metadata.image_height = readInt("image_height", 0);
    metadata.keyframes = readInt("keyframes", 0);
    metadata.frames_processed = static_cast<unsigned long long>(readInt("frames_processed", 0));
    metadata.frames_tracked = static_cast<unsigned long long>(readInt("frames_tracked", 0));
    return true;
}

} // namespace slam
