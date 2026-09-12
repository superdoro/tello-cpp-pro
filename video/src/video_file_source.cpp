#include "video/video_file_source.hpp"

#include <chrono>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "common/logging.hpp"

namespace video {

struct VideoFileSource::Impl {
    VideoFileConfig config;
    cv::VideoCapture capture;
    std::uint64_t index = 0;
    double fps = 0.0;
    std::chrono::steady_clock::time_point playback_start{};
};

VideoFileSource::VideoFileSource(VideoFileConfig config) : impl_(std::make_unique<Impl>()) {
    impl_->config = std::move(config);
    if (impl_->config.frame_stride < 1) impl_->config.frame_stride = 1;
}

VideoFileSource::~VideoFileSource() = default;

bool VideoFileSource::open() {
    if (!impl_->capture.open(impl_->config.path)) {
        common::logError("VideoFileSource", "cannot open video: " + impl_->config.path);
        return false;
    }
    impl_->fps = impl_->capture.get(cv::CAP_PROP_FPS);
    // A raw .h264 elementary stream (what tools/record_video writes) carries
    // no frame rate; assume the Tello's nominal 30fps so timestamps advance
    // at a plausible rate for SLAM.
    if (!(impl_->fps > 1.0) || impl_->fps > 240.0) impl_->fps = 30.0;
    impl_->index = 0;
    impl_->playback_start = std::chrono::steady_clock::now();
    return true;
}

void VideoFileSource::close() { impl_->capture.release(); }

bool VideoFileSource::isOpen() const { return impl_->capture.isOpened(); }

double VideoFileSource::nominalFps() const { return impl_->fps; }

std::uint64_t VideoFileSource::frameCount() const {
    const double count = impl_->capture.get(cv::CAP_PROP_FRAME_COUNT);
    return count > 0 ? static_cast<std::uint64_t>(count) : 0;
}

std::optional<Frame> VideoFileSource::nextFrame(int /*timeoutMs*/) {
    if (!impl_->capture.isOpened()) return std::nullopt;

    cv::Mat image;
    for (int i = 0; i < impl_->config.frame_stride; ++i) {
        if (!impl_->capture.read(image) || image.empty()) {
            close();  // end of stream: isOpen() now reports it, no frame follows
            return std::nullopt;
        }
    }

    if (impl_->config.target_width > 0 && impl_->config.target_height > 0 &&
        (image.cols != impl_->config.target_width || image.rows != impl_->config.target_height)) {
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(impl_->config.target_width, impl_->config.target_height),
                   0, 0, cv::INTER_AREA);
        image = std::move(resized);
    }

    Frame frame;
    frame.image = std::move(image);
    frame.index = impl_->index++;
    // Derive the timestamp from the frame index rather than from
    // CAP_PROP_POS_MSEC: raw elementary streams report 0 for position, and a
    // non-monotonic timestamp makes ORB-SLAM3's constant-velocity motion
    // model produce nonsense.
    frame.seconds = static_cast<double>(frame.index * impl_->config.frame_stride) / impl_->fps;
    frame.stamp = std::chrono::steady_clock::now();

    if (impl_->config.realtime_pacing) {
        const auto target = impl_->playback_start + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::duration<double>(frame.seconds));
        std::this_thread::sleep_until(target);
    }
    return frame;
}

} // namespace video
