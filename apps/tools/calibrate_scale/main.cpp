// Measures the one number a monocular map cannot know about itself: how many
// metres one SLAM unit is.
//
// Method: localize in the map, mark two positions a known real distance
// apart, and divide. Carrying the drone by hand between two marks on the
// floor is accurate enough and far safer than flying the measurement - and
// the `go`/`rc` commands are too imprecise to serve as ground truth anyway.
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
        "Measure a map's scale in metres per SLAM unit.\n\n"
        "  calibrate_scale --map <map.osa> --drone --distance <m>\n\n"
        "  --map <map.osa>    map to calibrate (its metadata is updated in place)\n"
        "  --drone            localize from the live stream\n"
        "  --video <file>     localize from a recording instead\n"
        "  --ip <addr>        drone address  [192.168.10.1]\n"
        "  --distance <m>     real distance between the two marks\n"
        "  --set-scale <m>    write an already-measured scale and exit (no drone needed)\n"
        "  --camera <yaml>    camera intrinsics [config/tello_camera.yaml]\n"
        "  --no-viewer        run without the Pangolin window\n\n"
        "In the preview window:  a = mark point A,  b = mark point B and record a\n"
        "sample,  Enter = finish and write the result,  Esc = quit without saving.\n";
}

double magnitude(const common::Vector3& a, const common::Vector3& b) {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
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
    // Apply an already-measured scale and exit - no drone, no map load.
    const double presetScale = args.getDouble("set-scale", 0.0);
    if (presetScale > 0.0) {
        slam::MapMetadata metadata;
        slam::loadMapMetadata(mapPath, metadata);
        metadata.scale_metres_per_unit = presetScale;
        metadata.scale_calibrated = true;
        if (!slam::saveMapMetadata(mapPath, metadata)) return 1;
        std::cout << "\nScale set to " << std::fixed << std::setprecision(4) << presetScale
                  << " m/unit in " << slam::metadataPathForMap(mapPath) << "\n\n";
        return 0;
    }

    const double knownDistance = args.getDouble("distance", 0.0);
    if (knownDistance <= 0.0) {
        std::cerr << "error: --distance <metres> is required\n\n";
        printUsage();
        return 1;
    }

    const std::string cameraPath = args.get("camera", "config/tello_camera.yaml");
    slam::CameraConfigInfo camera;
    if (!slam::readCameraConfig(cameraPath, camera)) return 1;

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
    // Deliberately 1.0: this tool must see raw SLAM units, not units already
    // scaled by a previous (possibly wrong) calibration.
    config.scale_metres_per_unit = 1.0;

    slam::OrbSlam3Localizer localizer;
    if (!localizer.initialize(config)) {
        if (feed.drone) feed.drone->disconnect();
        return 1;
    }

    std::signal(SIGINT, handleSigint);

    std::cout << "\nMove the drone between two marks " << knownDistance
              << " m apart.\n"
                 "  a = mark A    b = mark B (records a sample)    Enter = save    Esc = quit\n\n";

    std::vector<double> samples;
    bool haveA = false;
    common::Vector3 pointA{};
    bool save = false;

    while (!g_interrupted) {
        auto frame = feed.source->nextFrame(1000);
        if (!frame) {
            if (!feed.source->isOpen()) break;
            // Keep the window and the keyboard alive, or Esc has nowhere to go.
            const int stalledKey = app::showVideoStalled(
                "calibrate_scale", feed.source->healthSummary(),
                "Esc = quit without saving    Enter = save what has been measured");
            if (stalledKey == 27) break;
            if (stalledKey == 13 || stalledKey == 10) {
                save = true;
                break;
            }
            continue;
        }

        const common::PoseEstimate pose = localizer.processFrame(*frame);
        const bool tracked = pose.state == common::TrackingState::Ok;

        std::ostringstream hud;
        hud << "samples " << samples.size();
        if (haveA) hud << "  A marked";
        if (!samples.empty()) {
            double sum = 0.0;
            for (double sample : samples) sum += sample;
            hud << "  scale " << std::fixed << std::setprecision(4)
                << (sum / static_cast<double>(samples.size())) << " m/unit";
        }

        cv::Mat display = app::drawPoseOverlay(frame->image, pose, hud.str());
        cv::putText(display, "a = mark A   b = mark B   Enter = SAVE   Esc = discard",
                     {12, display.rows - 14}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                     cv::Scalar(200, 200, 200), 1);
        cv::imshow("calibrate_scale", display);
        const int key = cv::waitKey(1);

        if (key == 27) break;
        if (key == 13 || key == 10) {
            save = true;
            break;
        }
        if ((key == 'a' || key == 'b') && !tracked) {
            common::logWarn("calibrate_scale", "not tracking - cannot mark a point");
            continue;
        }
        if (key == 'a') {
            pointA = pose.pose.position;
            haveA = true;
            std::cout << "  A marked\n";
        } else if (key == 'b') {
            if (!haveA) {
                common::logWarn("calibrate_scale", "mark point A first");
                continue;
            }
            const double slamDistance = magnitude(pose.pose.position, pointA);
            if (slamDistance < 1e-4) {
                common::logWarn("calibrate_scale", "A and B are the same point - ignoring");
                continue;
            }
            const double scale = knownDistance / slamDistance;
            samples.push_back(scale);
            haveA = false;
            std::cout << "  sample " << samples.size() << ": " << std::fixed
                      << std::setprecision(4) << slamDistance << " units = " << knownDistance
                      << " m  ->  " << scale << " m/unit\n";
        }
    }

    localizer.shutdown();
    feed.source->close();
    if (feed.drone) {
        feed.drone->enableVideoStream(false);
        feed.drone->disconnect();
    }
    cv::destroyAllWindows();

    if (samples.empty()) {
        std::cout << "\nNo samples taken, nothing to save.\n";
        return 1;
    }

    double sum = 0.0;
    for (double sample : samples) sum += sample;
    const double mean = sum / static_cast<double>(samples.size());

    double variance = 0.0;
    for (double sample : samples) variance += (sample - mean) * (sample - mean);
    const double stddev =
        samples.size() > 1 ? std::sqrt(variance / static_cast<double>(samples.size() - 1)) : 0.0;

    std::cout << "\nScale: " << std::fixed << std::setprecision(4) << mean << " m/unit";
    if (samples.size() > 1) {
        std::cout << "  (n=" << samples.size() << ", sd=" << stddev << ")";
        if (stddev > 0.1 * mean) {
            std::cout << "\nWARNING: samples disagree by more than 10% - the tracking was "
                         "probably marginal. Consider redoing this in a better-mapped area.";
        }
    }
    std::cout << "\n";

    // Measuring this costs a flight's worth of setup, so a mistyped key must
    // never be the end of it: print what was measured and exactly how to
    // apply it, rather than discarding the numbers.
    if (!save) {
        std::cout << "\nNOT saved - the session ended with Esc or Ctrl-C rather than Enter.\n"
                  << "The measurement above is still good. To apply it without re-measuring:\n\n"
                  << "  " << args.program() << " --map " << mapPath << " --set-scale "
                  << std::fixed << std::setprecision(4) << mean << "\n\n";
        return 1;
    }

    slam::MapMetadata metadata;
    slam::loadMapMetadata(mapPath, metadata);  // keep whatever provenance exists
    metadata.scale_metres_per_unit = mean;
    metadata.scale_calibrated = true;
    if (!slam::saveMapMetadata(mapPath, metadata)) return 1;

    std::cout << "Written to " << slam::metadataPathForMap(mapPath) << "\n\n";
    return 0;
}
