// Compares an estimated trajectory against ground truth, the standard way:
// fit the best similarity transform between them (Umeyama) and report the
// residual as absolute trajectory error.
//
// Two things make this worth having in the repo rather than in a notebook.
// First, the fitted SCALE is exactly the metres-per-SLAM-unit that
// tools/calibrate_scale measures by hand - so on synthetic data the scale
// calibration can be checked against a known answer. Second, the residual
// separates "the map is wrong" from "the map is right but at the wrong
// scale", which are otherwise easy to confuse.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "common/cli.hpp"
#include "slam/map_metadata.hpp"

namespace {

// timestamp -> position
using Track = std::map<double, Eigen::Vector3d>;

bool readTum(const std::string& path, Track& track) {
    std::ifstream input(path);
    if (!input) {
        std::cerr << "error: cannot read " << path << "\n";
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        double t, x, y, z, qx, qy, qz, qw;
        if (!(fields >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
        track[t] = Eigen::Vector3d(x, y, z);
    }
    return !track.empty();
}

// Nearest ground-truth sample within `tolerance` seconds.
bool nearest(const Track& track, double timestamp, double tolerance, Eigen::Vector3d& out) {
    auto it = track.lower_bound(timestamp);
    double bestError = tolerance;
    bool found = false;
    for (int step = 0; step < 2; ++step) {
        if (step == 1) {
            if (it == track.begin()) break;
            --it;
        }
        if (it == track.end()) continue;
        const double error = std::abs(it->first - timestamp);
        if (error <= bestError) {
            bestError = error;
            out = it->second;
            found = true;
        }
    }
    return found;
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || argc == 1) {
        std::cout << "Compare an estimated trajectory with ground truth.\n\n"
                     "  --estimate <file.txt>   trajectory from map_builder (TUM)\n"
                     "  --truth <file.txt>      ground truth (TUM)\n"
                     "  --tolerance <s>         timestamp match window  [0.02]\n"
                     "  --write-scale <map.osa> record the fitted scale in the map's metadata\n";
        return argc == 1 ? 1 : 0;
    }

    Track estimate;
    Track truth;
    if (!readTum(args.get("estimate"), estimate)) return 1;
    if (!readTum(args.get("truth"), truth)) return 1;

    const double tolerance = args.getDouble("tolerance", 0.02);

    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
    for (const auto& [timestamp, position] : estimate) {
        Eigen::Vector3d truthPosition;
        if (!nearest(truth, timestamp, tolerance, truthPosition)) continue;
        source.push_back(position);
        target.push_back(truthPosition);
    }

    if (source.size() < 3) {
        std::cerr << "error: only " << source.size()
                  << " timestamps matched - are the two files from the same run?\n";
        return 1;
    }

    Eigen::Matrix3Xd sourceMatrix(3, source.size());
    Eigen::Matrix3Xd targetMatrix(3, target.size());
    for (std::size_t i = 0; i < source.size(); ++i) {
        sourceMatrix.col(static_cast<Eigen::Index>(i)) = source[i];
        targetMatrix.col(static_cast<Eigen::Index>(i)) = target[i];
    }

    // Umeyama: the least-squares similarity transform taking source to target.
    const Eigen::Matrix4d transform = Eigen::umeyama(sourceMatrix, targetMatrix, true);
    const Eigen::Matrix3d scaledRotation = transform.block<3, 3>(0, 0);
    const double scale = std::cbrt(scaledRotation.determinant());

    double squaredSum = 0.0;
    double worst = 0.0;
    for (std::size_t i = 0; i < source.size(); ++i) {
        const Eigen::Vector3d mapped =
            scaledRotation * source[i] + transform.block<3, 1>(0, 3);
        const double error = (mapped - target[i]).norm();
        squaredSum += error * error;
        worst = std::max(worst, error);
    }
    const double rmse = std::sqrt(squaredSum / static_cast<double>(source.size()));

    double pathLength = 0.0;
    for (std::size_t i = 1; i < target.size(); ++i) {
        pathLength += (target[i] - target[i - 1]).norm();
    }

    std::cout << std::fixed << std::setprecision(4)
              << "\nMatched poses:      " << source.size() << "\n"
              << "Ground-truth path:  " << pathLength << " m\n"
              << "Fitted scale:       " << scale << " m per SLAM unit\n"
              << "ATE (RMSE):         " << rmse << " m\n"
              << "ATE (worst):        " << worst << " m\n"
              << "Relative error:     " << std::setprecision(2)
              << (pathLength > 0 ? 100.0 * rmse / pathLength : 0.0) << " % of path length\n\n";

    const std::string mapPath = args.get("write-scale");
    if (!mapPath.empty()) {
        slam::MapMetadata metadata;
        slam::loadMapMetadata(mapPath, metadata);
        metadata.scale_metres_per_unit = scale;
        metadata.scale_calibrated = true;
        if (!slam::saveMapMetadata(mapPath, metadata)) return 1;
        std::cout << "Recorded the fitted scale in " << slam::metadataPathForMap(mapPath)
                  << "\n\n";
    }
    return 0;
}
