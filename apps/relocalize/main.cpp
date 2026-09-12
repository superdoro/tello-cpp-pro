// Milestone (d): prove that a map built by map_builder can actually be used
// to localize - the step between "I have a map" and "I can navigate with it".
//
// Run it against the same video the map was built from (should relocalize
// almost immediately), then against a *different* recording of the same
// space, then live on the drone sitting on the floor. Only once the third
// works is it worth flying anything.
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>

#include <opencv2/highgui.hpp>

#include "app_support.hpp"
#include "common/cli.hpp"
#include "common/logging.hpp"
#include "core/event_bus.hpp"
#include "slam/map_metadata.hpp"
#include "slam/orb_slam3_localizer.hpp"

namespace {

volatile std::sig_atomic_t g_interrupted = 0;
void handleSigint(int) { g_interrupted = 1; }

void printUsage() {
    std::cout <<
        "Localize against an existing map, from a video file or live from the drone.\n\n"
        "  relocalize --map <map.osa> --video <file>   (offline check)\n"
        "  relocalize --map <map.osa> --drone          (live check)\n\n"
        "  --map <map.osa>       map to localize against\n"
        "  --video <file>        localize against a recorded video\n"
        "  --drone               localize against the live stream\n"
        "  --ip <addr>           drone address        [192.168.10.1]\n"
        "  --camera <yaml>       camera intrinsics    [config/tello_camera.yaml]\n"
        "  --vocabulary <file>   ORB vocabulary       [third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt]\n"
        "  --record <file>       (live) also save the raw stream\n"
        "  --realtime            (video) play at the recording's own frame rate\n"
        "  --freeze              switch to localization-only mode once settled\n"
        "                        (off by default - it measures far worse)\n"
        "  --no-viewer           run without the Pangolin window\n"
        "  --no-preview          run without the OpenCV camera window\n";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || argc == 1) {
        printUsage();
        return argc == 1 ? 1 : 0;
    }

    const std::string mapPath = args.get("map");
    if (mapPath.empty()) {
        std::cerr << "error: --map is required\n\n";
        printUsage();
        return 1;
    }
    if (args.get("video").empty() && !args.has("drone")) {
        std::cerr << "error: pass either --video <file> or --drone\n\n";
        printUsage();
        return 1;
    }

    const std::string cameraPath = args.get("camera", "config/tello_camera.yaml");
    slam::CameraConfigInfo camera;
    if (!slam::readCameraConfig(cameraPath, camera)) return 1;

    slam::MapMetadata metadata;
    const bool haveMetadata = slam::loadMapMetadata(mapPath, metadata);
    if (haveMetadata && metadata.image_width > 0 &&
        (metadata.image_width != camera.width || metadata.image_height != camera.height)) {
        common::logError("relocalize",
                          "this map was built at " + std::to_string(metadata.image_width) + "x" +
                              std::to_string(metadata.image_height) + " but the camera config is " +
                              std::to_string(camera.width) + "x" + std::to_string(camera.height) +
                              " - localization against it cannot work");
        return 1;
    }

    core::EventBus bus;
    app::FrameFeed feed = app::openFrameFeed(args, camera, bus);
    if (!feed.source) return 1;

    slam::LocalizerConfig config;
    config.mode = slam::LocalizerMode::Localization;
    config.vocabulary_path =
        args.get("vocabulary", "third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    config.camera_config_path = cameraPath;
    config.map_path = mapPath;
    config.enable_viewer = !args.has("no-viewer");
    if (args.has("freeze")) config.freeze_after_tracked_frames = 90;

    slam::OrbSlam3Localizer localizer;
    if (!localizer.initialize(config)) {
        if (feed.drone) feed.drone->disconnect();
        return 1;
    }

    std::signal(SIGINT, handleSigint);
    const bool preview = !args.has("no-preview");

    const auto startedAt = std::chrono::steady_clock::now();
    bool everTracked = false;
    std::chrono::steady_clock::time_point firstTrackAt{};
    std::uint64_t frames = 0;
    std::uint64_t trackedFrames = 0;

    std::cout << "\nLocalizing against " << mapPath << "\n"
              << (haveMetadata && metadata.scale_calibrated
                       ? "Scale is calibrated: positions below are metres.\n"
                       : "Scale is NOT calibrated: positions below are arbitrary SLAM units.\n")
              << "Ctrl-C to stop.\n\n";

    while (!g_interrupted) {
        auto frame = feed.source->nextFrame(1000);
        if (!frame) {
            if (!feed.source->isOpen()) break;
            if (preview && app::showVideoStalled("relocalize", feed.source->healthSummary(),
                                                  "Esc = quit") == 27) {
                break;
            }
            continue;
        }
        ++frames;

        const common::PoseEstimate pose = localizer.processFrame(*frame);
        if (pose.state == common::TrackingState::Ok) {
            ++trackedFrames;
            if (!everTracked) {
                everTracked = true;
                firstTrackAt = std::chrono::steady_clock::now();
                std::cout << "\nRelocalized after "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(firstTrackAt -
                                                                                    startedAt)
                                 .count()
                          << " ms (" << frames << " frames)\n\n";
            }
        }

        if (frames % 15 == 0) {
            std::cout << "\r  " << std::setw(15) << std::left
                      << app::trackingStateName(pose.state) << "  x " << std::fixed
                      << std::setprecision(2) << std::setw(7) << pose.pose.position.x << " y "
                      << std::setw(7) << pose.pose.position.y << " z " << std::setw(7)
                      << pose.pose.position.z << "   conf " << std::setprecision(2)
                      << pose.confidence << "    " << std::flush;
        }

        if (preview) {
            cv::imshow("relocalize", app::drawPoseOverlay(frame->image, pose));
            if (cv::waitKey(1) == 27) break;
        }
    }

    std::cout << "\n\n";
    if (!everTracked) {
        common::logError("relocalize",
                          "never relocalized. Most likely causes, in order: the camera is not "
                          "looking at a part of the mapped space; the camera config does not "
                          "match the one the map was built with; the lighting has changed a lot "
                          "since mapping.");
    } else {
        std::cout << "Tracked " << trackedFrames << " of " << frames << " frames ("
                  << std::fixed << std::setprecision(1)
                  << (100.0 * static_cast<double>(trackedFrames) /
                      static_cast<double>(std::max<std::uint64_t>(frames, 1)))
                  << "%).\n";
    }

    localizer.shutdown();
    feed.source->close();
    if (feed.drone) {
        feed.drone->enableVideoStream(false);
        feed.drone->disconnect();
    }
    return everTracked ? 0 : 1;
}
