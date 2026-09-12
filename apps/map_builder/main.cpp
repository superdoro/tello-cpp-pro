// Milestone (c)+(d): build an ORB-SLAM3 map from a recorded video and write
// it to disk, together with the keyframe trajectory the route will be
// authored from.
//
// Building the map offline rather than in flight is the whole point: mapping
// is the step that most needs retrying with different settings, and a Tello
// battery lasts about 8 minutes. Record the route once, then iterate here for
// free.
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common/cli.hpp"
#include "common/logging.hpp"
#include "slam/camera_config.hpp"
#include "slam/map_metadata.hpp"
#include "slam/orb_slam3_localizer.hpp"
#include "video/video_file_source.hpp"

namespace {

volatile std::sig_atomic_t g_interrupted = 0;
void handleSigint(int) { g_interrupted = 1; }

void printUsage() {
    std::cout <<
        "Build a SLAM map from a recorded video.\n\n"
        "  map_builder --video <file> --out <map.osa> [options]\n\n"
        "  --video <file>        input video (.h264 from tools/record, or any container)\n"
        "  --out <map.osa>       where to write the map\n"
        "  --extend <map.osa>    add this footage to an existing map instead of starting\n"
        "                        fresh, keeping its coordinate frame and scale (must be a\n"
        "                        different file from --out)\n"
        "  --camera <yaml>       camera intrinsics   [config/tello_camera.yaml]\n"
        "  --vocabulary <file>   ORB vocabulary      [third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt]\n"
        "  --trajectory <file>   keyframe trajectory [<map>_trajectory.txt]\n"
        "  --stride <n>          use every nth frame [1]\n"
        "  --scale <m>           metres per SLAM unit, if already known [uncalibrated]\n"
        "  --no-viewer           run without the Pangolin window (headless)\n"
        "  --preview             show the input frames in an OpenCV window\n";
}

std::string defaultTrajectoryPath(const std::string& mapPath) {
    std::filesystem::path path(mapPath);
    path.replace_extension();
    return path.string() + "_trajectory.txt";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || argc == 1) {
        printUsage();
        return argc == 1 ? 1 : 0;
    }

    const std::string videoPath = args.get("video");
    const std::string mapPath = args.get("out");
    if (videoPath.empty() || mapPath.empty()) {
        std::cerr << "error: --video and --out are both required\n\n";
        printUsage();
        return 1;
    }

    const std::string cameraPath = args.get("camera", "config/tello_camera.yaml");
    const std::string vocabularyPath =
        args.get("vocabulary", "third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    const std::string trajectoryPath =
        args.get("trajectory", defaultTrajectoryPath(mapPath));
    const bool preview = args.has("preview");

    slam::CameraConfigInfo camera;
    if (!slam::readCameraConfig(cameraPath, camera)) return 1;

    video::VideoFileConfig videoConfig;
    videoConfig.path = videoPath;
    videoConfig.frame_stride = args.getInt("stride", 1);
    // Force the video to the resolution the intrinsics were measured at.
    // Without this a map built from, say, a 1280x720 recording would be
    // geometrically wrong in a way that shows up only later, as a map the
    // drone cannot relocalize against.
    videoConfig.target_width = camera.width;
    videoConfig.target_height = camera.height;

    video::VideoFileSource source(videoConfig);
    if (!source.open()) return 1;

    slam::LocalizerConfig config;
    config.mode = slam::LocalizerMode::Mapping;
    config.vocabulary_path = vocabularyPath;
    config.camera_config_path = cameraPath;
    config.map_path = mapPath;
    config.enable_viewer = !args.has("no-viewer");
    config.scale_metres_per_unit = args.getDouble("scale", 1.0);
    config.source_description = videoPath;
    config.extend_map_path = args.get("extend");

    slam::OrbSlam3Localizer localizer;
    if (!localizer.initialize(config)) return 1;

    std::signal(SIGINT, handleSigint);

    const std::uint64_t totalFrames = source.frameCount();
    std::uint64_t processed = 0;
    std::uint64_t tracked = 0;

    std::cout << "\nBuilding map from " << videoPath << "\n"
              << "Press Ctrl-C to stop early and save what has been mapped so far.\n\n";

    while (!g_interrupted) {
        auto frame = source.nextFrame(1000);
        if (!frame) {
            if (!source.isOpen()) break;  // end of file
            continue;
        }

        const common::PoseEstimate pose = localizer.processFrame(*frame);
        ++processed;
        if (pose.state == common::TrackingState::Ok) ++tracked;

        if (processed % 30 == 0) {
            const auto stats = localizer.stats();
            std::cout << "\r  frame " << processed;
            if (totalFrames > 0) std::cout << "/" << totalFrames;
            std::cout << "  tracked " << std::fixed << std::setprecision(1)
                      << (100.0 * static_cast<double>(tracked) / static_cast<double>(processed))
                      << "%  map points " << stats.tracked_map_points << "        " << std::flush;
        }

        if (preview) {
            cv::Mat display = frame->image.clone();
            const bool ok = pose.state == common::TrackingState::Ok;
            cv::putText(display, ok ? "TRACKING" : "NOT TRACKING", {12, 28},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        ok ? cv::Scalar(80, 220, 80) : cv::Scalar(60, 60, 230), 2);
            cv::imshow("map_builder input", display);
            if (cv::waitKey(1) == 27) break;
        }
    }
    std::cout << "\n\n";

    if (tracked == 0) {
        common::logError("map_builder",
                          "tracking never initialized - no map to save. Monocular SLAM needs "
                          "sideways translation (not pure rotation) and a textured scene to "
                          "bootstrap; check the camera calibration matches this video too.");
        localizer.shutdown();
        return 1;
    }

    // Order matters: saveMap() shuts the system down to serialize the atlas,
    // after which no trajectory can be read out of it.
    localizer.saveTrajectory(trajectoryPath);

    const bool extending = !config.extend_map_path.empty();
    const int keyframesBefore = localizer.stats().map_keyframes;

    if (!localizer.saveMap()) return 1;

    const auto stats = localizer.stats();
    std::cout << "Map:        " << mapPath << "\n"
              << "Trajectory: " << trajectoryPath << "\n"
              << "Frames:     " << stats.frames_processed << " processed, "
              << stats.frames_tracked << " tracked ("
              << std::fixed << std::setprecision(1)
              << (100.0 * static_cast<double>(stats.frames_tracked) /
                  static_cast<double>(std::max<std::uint64_t>(stats.frames_processed, 1)))
              << "%)\n\n";

    if (extending && keyframesBefore < 50) {
        // saveMap() has already explained what happened; this is about the
        // exit status, so a script extending a map does not treat a
        // disconnected offcut as success.
        std::cout << "The extension did not attach to the existing map, so this file cannot\n"
                     "be used to fly the existing route. See the warning above.\n\n";
        return 1;
    }

    slam::MapMetadata metadata;
    if (slam::loadMapMetadata(mapPath, metadata) && !metadata.scale_calibrated) {
        std::cout << "NOTE: this map has no scale. Its coordinates are in arbitrary SLAM\n"
                     "units, so distances mean nothing yet. Run tools/calibrate_scale\n"
                     "before flying a route against it.\n\n";
    }
    return 0;
}
