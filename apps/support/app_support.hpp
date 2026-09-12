#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "common/cli.hpp"
#include "common/types.hpp"
#include "drivers/tello/i_drone.hpp"
#include "slam/camera_config.hpp"
#include "video/i_frame_source.hpp"

#include <opencv2/core.hpp>

namespace core {
class EventBus;
}

namespace app {

// Everything needed to feed frames to SLAM, from either a file or the drone.
// The drone is null for file sources; when present the caller is responsible
// for landing it.
struct FrameFeed {
    std::unique_ptr<video::IFrameSource> source;
    std::unique_ptr<drivers::tello::IDrone> drone;
    bool live = false;
};

// Builds a frame feed from `--video <file>` or `--drone [--ip <addr>]`,
// sizing frames to the resolution the given camera config was calibrated at.
//
// Sharing this between the offline and the live tools is what guarantees the
// two paths deliver identical images - the property the entire
// build-a-map-offline, fly-against-it-online plan rests on.
FrameFeed openFrameFeed(const common::CommandLine& args, const slam::CameraConfigInfo& camera,
                         core::EventBus& bus);

// Draws a tracking/pose overlay onto a copy of `image`.
cv::Mat drawPoseOverlay(const cv::Mat& image, const common::PoseEstimate& pose,
                         const std::string& extraLine = {});

const char* trackingStateName(common::TrackingState state);

// Keeps a preview window alive, and the keyboard responsive, while no frames
// are arriving.
//
// It exists because the obvious `if (!frame) continue;` is a trap: with no
// frame there is no imshow, so on a stalled feed the window is never even
// created and cv::waitKey is never called - Esc does nothing, and the tool
// looks hung with no way out but Ctrl-C. Returns the key pressed, or -1.
int showVideoStalled(const std::string& window, const std::string& reason,
                      const std::string& keyHelp = {});

// Keyboard piloting from an OpenCV preview window, for taking manual control
// away from an autonomous flight.
//
// cv::waitKey gives no key-UP event - only the stream of repeats the OS
// generates while a key is held - so each axis is zeroed once its key stops
// repeating. The timeout must exceed the OS auto-repeat interval or a genuine
// hold stutters to a stop; the cost is that a single tap keeps its velocity
// for up to that long. (apps/manual_control_gui uses SDL2 precisely because a
// real window gives true key-up events, but adding a second windowing system
// to this app for an emergency control is not a worthwhile trade.)
//
// Letters only, deliberately: arrow-key codes differ between highgui's GTK
// and Qt backends, and an emergency control must not depend on which one
// OpenCV happened to be built against.
class ManualPilot {
public:
    explicit ManualPilot(int speed = 25) : speed_(speed) {}

    // Returns true if the key was a piloting key.
    bool onKey(int key, std::chrono::steady_clock::time_point now);

    // The command to send right now, with stale axes zeroed.
    common::VelocityCommand velocity(std::chrono::steady_clock::time_point now) const;

    void stop();
    void setSpeed(int speed) { speed_ = speed; }

    static const char* keyHelp();

private:
    struct Axis {
        int value = 0;
        std::chrono::steady_clock::time_point last_seen{};
    };

    int speed_;
    Axis pitch_;
    Axis roll_;
    Axis throttle_;
    Axis yaw_;
};

} // namespace app
