#include "video/tello_video_source.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "common/logging.hpp"
#include "common/net/udp_socket.hpp"
#include "video/h264_decoder.hpp"

namespace video {

struct TelloVideoSource::Impl {
    TelloVideoConfig config;
    common::net::UdpSocket socket;
    H264Decoder decoder;
    std::jthread receive_thread;
    std::FILE* record_file = nullptr;

    std::mutex mutex;
    std::condition_variable cv;
    std::optional<Frame> latest;
    std::chrono::steady_clock::time_point first_frame_at{};
    bool have_first_frame = false;

    std::atomic<std::uint64_t> decoded{0};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> datagrams{0};
    std::atomic<std::uint64_t> bytes{0};
    std::atomic<bool> running{false};

    void receiveLoop(std::stop_token token);
    void publish(cv::Mat image);
};

void TelloVideoSource::Impl::publish(cv::Mat image) {
    if (config.target_width > 0 && config.target_height > 0 &&
        (image.cols != config.target_width || image.rows != config.target_height)) {
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(config.target_width, config.target_height), 0, 0,
                   cv::INTER_AREA);
        image = std::move(resized);
    }

    const auto now = std::chrono::steady_clock::now();

    Frame frame;
    frame.image = std::move(image);
    frame.stamp = now;
    frame.index = decoded.fetch_add(1);

    {
        std::lock_guard lock(mutex);
        if (!have_first_frame) {
            first_frame_at = now;
            have_first_frame = true;
        }
        frame.seconds = std::chrono::duration<double>(now - first_frame_at).count();
        if (latest.has_value()) dropped.fetch_add(1);
        latest = std::move(frame);
    }
    cv.notify_one();
}

void TelloVideoSource::Impl::receiveLoop(std::stop_token token) {
    std::vector<std::uint8_t> buffer(4096);
    std::vector<cv::Mat> decodedFrames;

    while (!token.stop_requested()) {
        const std::size_t received = socket.receiveInto(buffer.data(), buffer.size(), 500);
        if (received == 0) continue;

        datagrams.fetch_add(1, std::memory_order_relaxed);
        bytes.fetch_add(received, std::memory_order_relaxed);

        if (record_file) {
            std::fwrite(buffer.data(), 1, received, record_file);
        }

        decodedFrames.clear();
        decoder.decode(buffer.data(), received, decodedFrames);
        for (auto& image : decodedFrames) {
            publish(std::move(image));
        }
    }
}

TelloVideoSource::TelloVideoSource(TelloVideoConfig config) : impl_(std::make_unique<Impl>()) {
    impl_->config = std::move(config);
}

TelloVideoSource::~TelloVideoSource() { close(); }

bool TelloVideoSource::open() {
    if (!impl_->socket.bind(impl_->config.port)) {
        common::logError("TelloVideoSource",
                          "cannot bind UDP port " + std::to_string(impl_->config.port));
        return false;
    }
    impl_->socket.setReceiveBufferBytes(impl_->config.receive_buffer_bytes);

    if (!impl_->decoder.open()) {
        impl_->socket.close();
        return false;
    }

    if (!impl_->config.record_path.empty()) {
        impl_->record_file = std::fopen(impl_->config.record_path.c_str(), "wb");
        if (!impl_->record_file) {
            common::logWarn("TelloVideoSource",
                             "cannot open record file, continuing without recording: " +
                                 impl_->config.record_path);
        } else {
            common::logInfo("TelloVideoSource", "recording raw H264 to " + impl_->config.record_path);
        }
    }

    impl_->running = true;
    impl_->receive_thread =
        std::jthread([this](std::stop_token token) { impl_->receiveLoop(token); });
    return true;
}

void TelloVideoSource::close() {
    if (!impl_->running.exchange(false)) return;

    if (impl_->receive_thread.joinable()) {
        impl_->receive_thread.request_stop();
        impl_->receive_thread.join();
    }
    impl_->socket.close();
    impl_->decoder.close();
    if (impl_->record_file) {
        std::fclose(impl_->record_file);
        impl_->record_file = nullptr;
    }
    impl_->cv.notify_all();
}

bool TelloVideoSource::isOpen() const { return impl_->running.load(); }

std::uint64_t TelloVideoSource::decodedCount() const { return impl_->decoded.load(); }
std::uint64_t TelloVideoSource::droppedCount() const { return impl_->dropped.load(); }
std::uint64_t TelloVideoSource::receivedDatagrams() const { return impl_->datagrams.load(); }
std::uint64_t TelloVideoSource::receivedBytes() const { return impl_->bytes.load(); }
std::uint64_t TelloVideoSource::decoderErrors() const { return impl_->decoder.errorCount(); }

std::string TelloVideoSource::healthSummary() const {
    if (receivedDatagrams() == 0) {
        return "no UDP data on port " + std::to_string(impl_->config.port) +
               " - firewall, or the drone is not streaming";
    }
    return std::to_string(receivedDatagrams()) + " datagrams / " +
           std::to_string(receivedBytes() / 1024) + " KiB, " +
           std::to_string(decodedCount()) + " frames decoded, " +
           std::to_string(decoderErrors()) + " decode errors";
}

std::optional<Frame> TelloVideoSource::nextFrame(int timeoutMs) {
    std::unique_lock lock(impl_->mutex);
    if (!impl_->latest.has_value()) {
        impl_->cv.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [this] { return impl_->latest.has_value() || !impl_->running.load(); });
    }
    if (!impl_->latest.has_value()) return std::nullopt;

    Frame frame = std::move(*impl_->latest);
    impl_->latest.reset();
    return frame;
}

} // namespace video
