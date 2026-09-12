#include "control/mission_io.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <opencv2/core/persistence.hpp>

#include "common/logging.hpp"

namespace control {
namespace {

int millis(std::chrono::milliseconds value) { return static_cast<int>(value.count()); }

}  // namespace

bool saveMission(const std::string& path, const Mission& mission) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        common::logError("MissionIO", "cannot write mission to " + path);
        return false;
    }

    fs << "name" << mission.name;
    fs << "map_path" << mission.map_path;
    fs << "geofence_max_radius_m" << mission.config.geofence.max_radius_m;
    fs << "geofence_max_altitude_m" << mission.config.geofence.max_altitude_m;
    fs << "geofence_min_altitude_m" << mission.config.geofence.min_altitude_m;
    fs << "max_tracking_loss_ms" << millis(mission.config.max_tracking_loss);
    fs << "min_confidence" << mission.config.min_confidence;
    fs << "continuous" << (mission.config.continuous ? 1 : 0);
    fs << "lookahead_m" << mission.config.lookahead_m;
    fs << "lookahead_corner_limit_rad" << mission.config.lookahead_corner_limit_rad;
    fs << "lookahead_time_s" << mission.config.lookahead_time_s;
    fs << "lookahead_max_m" << mission.config.lookahead_max_m;
    fs << "corner_brake_time_s" << mission.config.corner_brake_time_s;
    fs << "corner_full_slow_rad" << mission.config.corner_full_slow_rad;
    fs << "corner_min_cruise_scale" << mission.config.corner_min_cruise_scale;

    fs << "waypoints" << "[";
    for (const auto& waypoint : mission.waypoints) {
        fs << "{";
        fs << "label" << waypoint.label;
        fs << "x" << waypoint.position.x;
        fs << "y" << waypoint.position.y;
        fs << "z" << waypoint.position.z;
        fs << "heading_rad" << waypoint.heading_rad;
        fs << "hold_heading" << (waypoint.hold_heading ? 1 : 0);
        fs << "position_tolerance_m" << waypoint.position_tolerance_m;
        fs << "heading_tolerance_rad" << waypoint.heading_tolerance_rad;
        fs << "dwell_ms" << millis(waypoint.dwell);
        fs << "timeout_ms" << millis(waypoint.timeout);
        fs << "}";
    }
    fs << "]";
    fs.release();

    common::logInfo("MissionIO", "wrote " + std::to_string(mission.waypoints.size()) +
                                      " waypoints to " + path);
    return true;
}

bool loadMission(const std::string& path, Mission& mission) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        common::logError("MissionIO", "cannot read mission from " + path);
        return false;
    }

    const auto readFloat = [&fs](const char* key, float fallback) {
        const cv::FileNode node = fs[key];
        return node.empty() ? fallback : static_cast<float>(node);
    };
    const auto readInt = [&fs](const char* key, int fallback) {
        const cv::FileNode node = fs[key];
        return node.empty() ? fallback : static_cast<int>(node);
    };
    const auto readString = [&fs](const char* key) {
        const cv::FileNode node = fs[key];
        return node.empty() ? std::string{} : static_cast<std::string>(node);
    };

    mission = Mission{};
    mission.name = readString("name");
    mission.map_path = readString("map_path");
    mission.config.geofence.max_radius_m = readFloat("geofence_max_radius_m", 15.0f);
    mission.config.geofence.max_altitude_m = readFloat("geofence_max_altitude_m", 3.0f);
    mission.config.geofence.min_altitude_m = readFloat("geofence_min_altitude_m", -1.0f);
    mission.config.max_tracking_loss =
        std::chrono::milliseconds(readInt("max_tracking_loss_ms", 4000));
    mission.config.min_confidence = readFloat("min_confidence", 0.15f);
    mission.config.continuous = readInt("continuous", 1) != 0;
    mission.config.lookahead_m = readFloat("lookahead_m", 1.2f);
    mission.config.lookahead_corner_limit_rad =
        readFloat("lookahead_corner_limit_rad", 0.6f);
    mission.config.lookahead_time_s = readFloat("lookahead_time_s", 0.5f);
    mission.config.lookahead_max_m = readFloat("lookahead_max_m", 3.0f);
    mission.config.corner_brake_time_s = readFloat("corner_brake_time_s", 1.5f);
    mission.config.corner_full_slow_rad = readFloat("corner_full_slow_rad", 1.2f);
    mission.config.corner_min_cruise_scale = readFloat("corner_min_cruise_scale", 0.25f);

    const cv::FileNode waypoints = fs["waypoints"];
    if (waypoints.empty() || !waypoints.isSeq()) {
        common::logError("MissionIO", "mission has no waypoints sequence: " + path);
        return false;
    }

    for (const auto& node : waypoints) {
        Waypoint waypoint;
        const auto nodeFloat = [&node](const char* key, float fallback) {
            const cv::FileNode child = node[key];
            return child.empty() ? fallback : static_cast<float>(child);
        };
        const auto nodeInt = [&node](const char* key, int fallback) {
            const cv::FileNode child = node[key];
            return child.empty() ? fallback : static_cast<int>(child);
        };
        const cv::FileNode label = node["label"];
        if (!label.empty()) waypoint.label = static_cast<std::string>(label);

        waypoint.position = {nodeFloat("x", 0.0f), nodeFloat("y", 0.0f), nodeFloat("z", 0.0f)};
        waypoint.heading_rad = nodeFloat("heading_rad", 0.0f);
        waypoint.hold_heading = nodeInt("hold_heading", 0) != 0;
        waypoint.position_tolerance_m = nodeFloat("position_tolerance_m", 0.25f);
        waypoint.heading_tolerance_rad = nodeFloat("heading_tolerance_rad", 0.35f);
        waypoint.dwell = std::chrono::milliseconds(nodeInt("dwell_ms", 300));
        waypoint.timeout = std::chrono::milliseconds(nodeInt("timeout_ms", 20000));
        mission.waypoints.push_back(waypoint);
    }

    common::logInfo("MissionIO", "loaded " + std::to_string(mission.waypoints.size()) +
                                      " waypoints from " + path);
    return true;
}

bool missionFromTrajectory(const std::string& trajectoryPath,
                            const TrajectorySamplingConfig& config,
                            const FrameAlignment& alignment, Mission& mission) {
    std::ifstream input(trajectoryPath);
    if (!input) {
        common::logError("MissionIO", "cannot read trajectory: " + trajectoryPath);
        return false;
    }

    mission = Mission{};
    mission.name = "from_trajectory";

    bool haveLast = false;
    common::Vector3 last{};
    std::string line;
    std::size_t lineNumber = 0;
    std::size_t poses = 0;
    int sampled = 0;

    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream fields(line);
        double timestamp = 0.0;
        double tx = 0.0, ty = 0.0, tz = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        if (!(fields >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            common::logWarn("MissionIO", "skipping malformed trajectory line " +
                                              std::to_string(lineNumber));
            continue;
        }
        ++poses;

        const auto scale = static_cast<float>(config.scale_metres_per_unit);
        const common::Vector3 position{static_cast<float>(tx) * scale,
                                        static_cast<float>(ty) * scale,
                                        static_cast<float>(tz) * scale};

        if (haveLast && distance(position, last) < config.spacing_m) continue;

        last = position;
        haveLast = true;

        if (sampled++ < config.skip_first) continue;
        if (config.max_waypoints > 0 &&
            static_cast<int>(mission.waypoints.size()) >= config.max_waypoints) {
            break;
        }

        Waypoint waypoint;
        waypoint.position = position;
        waypoint.position_tolerance_m = config.position_tolerance_m;
        waypoint.hold_heading = config.hold_heading;
        if (config.hold_heading) {
            const common::Quaternion orientation{static_cast<float>(qw), static_cast<float>(qx),
                                                  static_cast<float>(qy), static_cast<float>(qz)};
            waypoint.heading_rad = alignment.headingInMap(orientation);
        }
        waypoint.label = "kf_" + std::to_string(poses);
        mission.waypoints.push_back(waypoint);
    }

    if (mission.waypoints.empty()) {
        common::logError("MissionIO", "trajectory produced no waypoints: " + trajectoryPath);
        return false;
    }

    // Size the geofence to the route. Altitude is measured along the map's up
    // axis, the same way MissionPlanner checks it.
    const common::Vector3& up = alignment.map_up;
    const float upLength = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
    float maxRadius = 0.0f;
    float minAltitude = 0.0f;
    float maxAltitude = 0.0f;
    bool first = true;
    for (const auto& waypoint : mission.waypoints) {
        const common::Vector3& p = waypoint.position;
        maxRadius = std::max(maxRadius, std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
        if (upLength > 1e-6f) {
            const float altitude = (p.x * up.x + p.y * up.y + p.z * up.z) / upLength;
            minAltitude = first ? altitude : std::min(minAltitude, altitude);
            maxAltitude = first ? altitude : std::max(maxAltitude, altitude);
            first = false;
        }
    }
    mission.config.geofence.max_radius_m = maxRadius + config.geofence_margin_m;
    mission.config.geofence.max_altitude_m = maxAltitude + config.geofence_margin_m;
    mission.config.geofence.min_altitude_m = minAltitude - config.geofence_margin_m;

    common::logInfo("MissionIO",
                     "geofence sized to the route: radius " +
                         std::to_string(mission.config.geofence.max_radius_m) + " m, altitude " +
                         std::to_string(mission.config.geofence.min_altitude_m) + " to " +
                         std::to_string(mission.config.geofence.max_altitude_m) + " m");

    common::logInfo("MissionIO", "sampled " + std::to_string(mission.waypoints.size()) +
                                      " waypoints from " + std::to_string(poses) +
                                      " keyframes in " + trajectoryPath);
    return true;
}

} // namespace control
