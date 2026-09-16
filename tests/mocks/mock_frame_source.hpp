#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "video/i_frame_source.hpp"

namespace mocks {

// An IFrameSource that plays back a scripted list of frames.
//
// The stall mode matters as much as the frames: a feed that goes quiet is one
// of the few failure modes that the pose path cannot see (the planner simply
// never gets a pose), so every consumer needs to be tested against it.
class MockFrameSource : public video::IFrameSource {
public:
    bool open() override {
        ++open_calls;
        opened = open_succeeds;
        return open_succeeds;
    }

    void close() override {
        ++close_calls;
        opened = false;
    }

    bool isOpen() const override { return opened; }

    std::optional<video::Frame> nextFrame(int timeoutMs) override {
        last_timeout_ms = timeoutMs;
        ++next_frame_calls;

        if (stalled) return std::nullopt;
        if (cursor >= frames.size()) {
            // A file source reports end-of-stream by closing; a live source
            // stays open and simply has nothing yet. Callers distinguish the
            // two with isOpen(), so the mock must model both.
            if (close_at_end) opened = false;
            return std::nullopt;
        }
        return frames[cursor++];
    }

    double nominalFps() const override { return fps; }

    std::string healthSummary() const override { return health; }

    // Appends a frame with sensible defaults, so a test that does not care
    // about pixels does not have to build a cv::Mat.
    void appendFrame(const cv::Mat& image, double seconds) {
        video::Frame frame;
        frame.image = image;
        frame.seconds = seconds;
        frame.index = frames.size();
        frame.stamp = common::Timestamp{} + std::chrono::milliseconds(
                                                static_cast<long long>(seconds * 1000.0));
        frames.push_back(std::move(frame));
    }

    bool exhausted() const { return cursor >= frames.size(); }
    void rewind() { cursor = 0; }

    // --- scripted behaviour ----------------------------------------------
    std::vector<video::Frame> frames;
    bool open_succeeds = true;
    bool close_at_end = true;
    bool stalled = false;     // flip mid-test to simulate a dead feed
    double fps = 30.0;
    std::string health;

    // --- recorded state ---------------------------------------------------
    bool opened = false;
    std::size_t cursor = 0;
    int open_calls = 0;
    int close_calls = 0;
    int next_frame_calls = 0;
    int last_timeout_ms = 0;
};

} // namespace mocks
