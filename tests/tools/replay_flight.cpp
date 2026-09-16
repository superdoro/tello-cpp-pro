// Replays a recorded trajectory through the flight control stack, with no
// SLAM and no video, and prints the stick commands it would have produced.
//
// It exists because the obvious end-to-end check - fly the same route twice
// and diff the commands - cannot work on this project: ORB-SLAM3's
// relocalization and its LocalMapping/LoopClosing threads are not
// deterministic, and two dry runs against the SAME map differ in most of their
// commands. There is no reproducible baseline to compare against.
//
// Feeding recorded poses straight into MissionPlanner + ObstacleAvoidance +
// PidFlightController removes every nondeterministic component while keeping
// the whole part under test, so `--avoid` and no `--avoid` become directly
// comparable, byte for byte.

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "common/cli.hpp"
#include "common/logging.hpp"
#include "common/types.hpp"
#include "control/mission_io.hpp"
#include "control/mission_planner.hpp"
#include "control/obstacle_avoidance.hpp"
#include "control/pid_flight_controller.hpp"

namespace {

struct Sample {
    double seconds = 0.0;
    common::Pose pose;
};

bool readTum(const std::string& path, std::vector<Sample>& out) {
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
        Sample sample;
        sample.seconds = t;
        sample.pose.position = {static_cast<float>(x), static_cast<float>(y),
                                 static_cast<float>(z)};
        sample.pose.orientation = {static_cast<float>(qw), static_cast<float>(qx),
                                    static_cast<float>(qy), static_cast<float>(qz)};
        out.push_back(sample);
    }
    if (out.empty()) std::cerr << "error: no usable poses in " << path << "\n";
    return !out.empty();
}

void printUsage() {
    std::cout <<
        "replay_flight - drive the control stack from a recorded trajectory\n\n"
        "  --trajectory <file>   TUM poses: timestamp tx ty tz qx qy qz qw (required)\n"
        "  --mission <file>      mission YAML (required)\n"
        "  --scale <m>           metres per trajectory unit          [1.0]\n"
        "  --avoid               enable the obstacle avoidance layer [off]\n"
        "  --cruise <n>          cruise feedforward command          [0]\n"
        "  --max-speed <n>       horizontal stick cap                [25]\n"
        "  --verbose             also print state, clearance and cruise scale\n";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || !args.has("trajectory") || !args.has("mission")) {
        printUsage();
        return args.has("help") ? 0 : 1;
    }
    common::setMinLogLevel(common::LogLevel::Warn);

    std::vector<Sample> samples;
    if (!readTum(args.get("trajectory"), samples)) return 1;

    control::Mission mission;
    if (!control::loadMission(args.get("mission"), mission)) return 1;

    const float scale = static_cast<float>(args.getDouble("scale", 1.0));
    for (auto& sample : samples) {
        sample.pose.position.x *= scale;
        sample.pose.position.y *= scale;
        sample.pose.position.z *= scale;
    }

    control::FlightControllerConfig controllerConfig;
    controllerConfig.cruise_command = args.getInt("cruise", 0);
    controllerConfig.max_horizontal_command = args.getInt("max-speed", 25);

    control::PidFlightController controller(controllerConfig);
    control::MissionPlanner planner;
    planner.setAlignment(controllerConfig.alignment);
    planner.loadMission(mission);

    control::AvoidanceConfig avoidConfig = mission.config.avoidance;
    if (args.has("avoid")) avoidConfig.enabled = true;
    control::ObstacleAvoidance avoidance(avoidConfig);
    avoidance.setAlignment(controllerConfig.alignment);
    avoidance.setGeofence(mission.config.geofence);

    const bool verbose = args.has("verbose");

    // A synthetic clock driven by the trajectory's own timestamps, so the run
    // does not depend on how fast the machine is.
    const auto epoch = std::chrono::steady_clock::time_point{};
    const auto stampOf = [&epoch](double seconds) {
        return epoch + std::chrono::microseconds(static_cast<long long>(seconds * 1e6));
    };

    planner.start();
    auto previousAt = stampOf(samples.front().seconds);
    int emitted = 0;

    for (const auto& sample : samples) {
        const auto now = stampOf(sample.seconds);

        common::PoseEstimate pose;
        pose.pose = sample.pose;
        pose.pose.stamp = now;
        pose.state = common::TrackingState::Ok;
        pose.confidence = 1.0f;

        planner.onPoseUpdate(pose, now);
        const auto status = planner.status();
        if (status == control::MissionStatus::Complete ||
            status == control::MissionStatus::Aborted) {
            break;
        }

        common::VelocityCommand command;
        control::AvoidanceDecision decision;
        if (status == control::MissionStatus::Running) {
            if (const auto target = planner.currentTarget()) {
                const std::chrono::duration<double> dt = now - previousAt;

                avoidance.setSpeed(planner.speed());
                // No perception in this harness: the snapshot is always absent,
                // which is exactly the case the identity guarantee covers.
                decision = avoidance.evaluate(pose, *target, std::nullopt, now);

                control::Waypoint adjusted = *target;
                adjusted.position.x += decision.target_offset_map.x;
                adjusted.position.y += decision.target_offset_map.y;
                adjusted.position.z += decision.target_offset_map.z;

                const common::Vector3 pathOffset = planner.pathOffset();
                controller.setCruiseScale(planner.cruiseScale() * decision.cruise_scale);
                controller.setPathOffset({pathOffset.x * decision.path_offset_scale,
                                           pathOffset.y * decision.path_offset_scale,
                                           pathOffset.z * decision.path_offset_scale});
                command = controller.computeCommand(pose, adjusted, dt);
                if (decision.hold) {
                    controller.reset();
                    command = {};
                }
            }
        } else {
            controller.reset();
        }
        previousAt = now;

        std::cout << "rc " << command.roll << " " << command.pitch << " " << command.throttle
                  << " " << command.yaw;
        if (verbose) {
            std::cout << "  " << control::avoidanceStateName(decision.state) << "  cruise "
                      << std::fixed << std::setprecision(2) << decision.cruise_scale << "  wp "
                      << planner.currentIndex();
        }
        std::cout << "\n";
        ++emitted;
    }

    std::cerr << "replay_flight: " << emitted << " commands, mission "
              << (planner.status() == control::MissionStatus::Complete ? "COMPLETE"
                                                                        : "INCOMPLETE")
              << ", reached " << planner.currentIndex() << " of " << planner.waypointCount()
              << " waypoints\n";
    return planner.status() == control::MissionStatus::Complete ? 0 : 1;
}
