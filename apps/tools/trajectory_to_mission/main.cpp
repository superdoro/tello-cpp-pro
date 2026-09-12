// Turns the keyframe trajectory a mapping run produced into a flight route.
//
// This is how routes should be authored for this project. Waypoints invented
// by hand may sit somewhere the drone cannot see any mapped landmarks, and it
// will hold position forever waiting to relocalize. Every waypoint sampled
// from the mapping trajectory is, by construction, a pose the camera has
// already localized from.
#include <iostream>

#include "common/cli.hpp"
#include "common/logging.hpp"
#include "control/mission_io.hpp"
#include "slam/map_metadata.hpp"

namespace {

void printUsage() {
    std::cout <<
        "Convert a SLAM keyframe trajectory into a mission route.\n\n"
        "  trajectory_to_mission --map <map.osa> --out <mission.yaml> [options]\n\n"
        "  --map <map.osa>       map the trajectory belongs to (supplies the scale)\n"
        "  --trajectory <file>   trajectory  [the one recorded in the map's metadata]\n"
        "  --out <mission.yaml>  route to write\n"
        "  --spacing <m>         distance between waypoints  [0.6]\n"
        "  --tolerance <m>       arrival tolerance           [0.25]\n"
        "  --free-heading        do not command a heading at each waypoint\n"
        "  --skip <n>            drop the first n waypoints (taxi-out footage)\n"
        "  --limit <n>           keep at most n waypoints\n"
        "  --margin <m>          geofence margin around the route  [3.0]\n"
        "  --name <text>         mission name\n";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || argc == 1) {
        printUsage();
        return argc == 1 ? 1 : 0;
    }

    const std::string mapPath = args.get("map");
    const std::string outPath = args.get("out");
    if (mapPath.empty() || outPath.empty()) {
        std::cerr << "error: --map and --out are both required\n\n";
        printUsage();
        return 1;
    }

    slam::MapMetadata metadata;
    if (!slam::loadMapMetadata(mapPath, metadata)) {
        common::logError("trajectory_to_mission",
                          "no metadata sidecar for " + mapPath + " - was it built by map_builder?");
        return 1;
    }

    const std::string trajectoryPath = args.get("trajectory", metadata.trajectory_file);
    if (trajectoryPath.empty()) {
        common::logError("trajectory_to_mission",
                          "the map records no trajectory file; pass --trajectory explicitly");
        return 1;
    }

    if (!metadata.scale_calibrated) {
        common::logWarn("trajectory_to_mission",
                         "map scale is uncalibrated - the route's distances will be in SLAM "
                         "units, and autonomous_mission will refuse to fly it until "
                         "tools/calibrate_scale has run");
    }

    control::TrajectorySamplingConfig sampling;
    sampling.spacing_m = static_cast<float>(args.getDouble("spacing", 0.6));
    sampling.position_tolerance_m = static_cast<float>(args.getDouble("tolerance", 0.25));
    sampling.hold_heading = !args.has("free-heading");
    sampling.scale_metres_per_unit = metadata.scale_metres_per_unit;
    sampling.skip_first = args.getInt("skip", 0);
    sampling.max_waypoints = args.getInt("limit", 0);
    sampling.geofence_margin_m = static_cast<float>(args.getDouble("margin", 3.0));

    control::FrameAlignment alignment;
    control::Mission mission;
    if (!control::missionFromTrajectory(trajectoryPath, sampling, alignment, mission)) return 1;

    mission.name = args.get("name", "route_from_" + trajectoryPath);
    mission.map_path = mapPath;

    if (!control::saveMission(outPath, mission)) return 1;

    std::cout << "\nWrote " << mission.waypoints.size() << " waypoints to " << outPath << "\n"
              << "Review it before flying: the trajectory includes wherever the camera\n"
              << "wandered while mapping, including any backtracking.\n\n";
    return 0;
}
