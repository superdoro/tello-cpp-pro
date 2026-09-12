#include "app_support.hpp"

#include <iomanip>
#include <sstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common/logging.hpp"
#include "core/event_bus.hpp"
#include "drivers/tello/tello_driver.hpp"
#include "video/tello_video_source.hpp"
#include "video/video_file_source.hpp"

namespace app {

const char* trackingStateName(common::TrackingState state) {
    switch (state) {
        case common::TrackingState::Ok: return "OK";
        case common::TrackingState::Lost: return "LOST";
        case common::TrackingState::Recovering: return "RECOVERING";
        case common::TrackingState::NotInitialized: return "NOT INITIALIZED";
    }
    return "?";
}

FrameFeed openFrameFeed(const common::CommandLine& args, const slam::CameraConfigInfo& camera,
                         core::EventBus& bus) {
    FrameFeed feed;

    const std::string videoPath = args.get("video");
    if (!videoPath.empty()) {
        video::VideoFileConfig config;
        config.path = videoPath;
        config.frame_stride = args.getInt("stride", 1);
        config.target_width = camera.width;
        config.target_height = camera.height;
        // Replaying a recording to test the live path is only meaningful at
        // the speed the live path will see.
        config.realtime_pacing = args.has("realtime");
        feed.source = std::make_unique<video::VideoFileSource>(config);
        if (!feed.source->open()) feed.source.reset();
        return feed;
    }

    auto driver = std::make_unique<drivers::tello::TelloDriver>(
        drivers::tello::TelloConnectionConfig{.ip = args.get("ip", "192.168.10.1")}, bus);
    if (!driver->connect()) {
        common::logError("app", "cannot connect to the drone");
        return feed;
    }
    video::TelloVideoConfig config;
    config.record_path = args.get("record");
    config.target_width = camera.width;
    config.target_height = camera.height;

    // The video port is bound before `streamon` is sent, never after: a
    // datagram arriving at an unbound port earns an ICMP port-unreachable
    // reply, and some Tello firmware stops streaming when it gets one.
    auto source = std::make_unique<video::TelloVideoSource>(config);
    if (!source->open()) {
        driver->disconnect();
        return feed;
    }
    if (!driver->enableVideoStream(true)) {
        common::logError("app", "drone refused streamon");
        source->close();
        driver->disconnect();
        return feed;
    }

    feed.source = std::move(source);
    feed.drone = std::move(driver);
    feed.live = true;
    return feed;
}

namespace {
// Longer than a typical OS auto-repeat interval (a few tens of ms), short
// enough that letting go still stops the drone promptly.
constexpr auto kManualHoldTimeout = std::chrono::milliseconds(500);
}  // namespace

const char* ManualPilot::keyHelp() {
    return "w/s fwd-back   a/d left-right   r/f up-down   q/e yaw";
}

bool ManualPilot::onKey(int key, std::chrono::steady_clock::time_point now) {
    const auto set = [&now](Axis& axis, int value) {
        axis.value = value;
        axis.last_seen = now;
    };
    switch (key) {
        case 'w': set(pitch_, speed_); return true;
        case 's': set(pitch_, -speed_); return true;
        case 'a': set(roll_, -speed_); return true;
        case 'd': set(roll_, speed_); return true;
        case 'r': set(throttle_, speed_); return true;
        case 'f': set(throttle_, -speed_); return true;
        case 'q': set(yaw_, -speed_); return true;
        case 'e': set(yaw_, speed_); return true;
        default: return false;
    }
}

common::VelocityCommand ManualPilot::velocity(std::chrono::steady_clock::time_point now) const {
    const auto live = [&now](const Axis& axis) {
        return now - axis.last_seen < kManualHoldTimeout ? axis.value : 0;
    };
    common::VelocityCommand command;
    command.pitch = live(pitch_);
    command.roll = live(roll_);
    command.throttle = live(throttle_);
    command.yaw = live(yaw_);
    return command;
}

void ManualPilot::stop() {
    pitch_ = {};
    roll_ = {};
    throttle_ = {};
    yaw_ = {};
}

int showVideoStalled(const std::string& window, const std::string& reason,
                      const std::string& keyHelp) {
    cv::Mat placeholder(320, 720, CV_8UC3, cv::Scalar(24, 22, 20));
    cv::putText(placeholder, "waiting for video", {24, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                 cv::Scalar(60, 60, 230), 2);

    // Wrap the reason by hand; it is usually one long diagnostic line.
    int y = 110;
    for (std::size_t start = 0; start < reason.size(); start += 60, y += 26) {
        cv::putText(placeholder, reason.substr(start, 60), {24, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                     cv::Scalar(200, 200, 200), 1);
    }
    if (!keyHelp.empty()) {
        cv::putText(placeholder, keyHelp, {24, 290}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                     cv::Scalar(160, 160, 160), 1);
    }

    cv::imshow(window, placeholder);
    return cv::waitKey(30);
}

cv::Mat drawPoseOverlay(const cv::Mat& image, const common::PoseEstimate& pose,
                         const std::string& extraLine) {
    cv::Mat display = image.clone();
    if (display.empty()) return display;

    const bool ok = pose.state == common::TrackingState::Ok;
    const cv::Scalar colour = ok ? cv::Scalar(80, 220, 80) : cv::Scalar(60, 60, 230);

    cv::rectangle(display, cv::Rect(0, 0, display.cols, 86), cv::Scalar(0, 0, 0), cv::FILLED);

    cv::putText(display, trackingStateName(pose.state), {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.75,
                colour, 2);

    std::ostringstream line;
    line << std::fixed << std::setprecision(2) << "x " << pose.pose.position.x << "  y "
         << pose.pose.position.y << "  z " << pose.pose.position.z << " m";
    cv::putText(display, ok ? line.str() : std::string("pose unavailable"), {12, 55},
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1);

    std::ostringstream second;
    second << "confidence " << std::fixed << std::setprecision(2) << pose.confidence;
    if (!extraLine.empty()) second << "   " << extraLine;
    cv::putText(display, second.str(), {12, 78}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(180, 180, 180), 1);

    return display;
}

} // namespace app
