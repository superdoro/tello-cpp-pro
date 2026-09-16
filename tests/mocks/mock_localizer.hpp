#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "common/camera_intrinsics.hpp"
#include "common/sparse_depth.hpp"
#include "slam/i_localizer.hpp"

namespace mocks {

// An ILocalizer that replays a scripted sequence of poses.
//
// This exists because the interesting cases are the ones a real flight will
// not reproduce on demand: tracking dropping out mid-route, a confident pose
// that is wrong, and a loop closure shifting the whole map frame underneath
// everything built in it. Each is one call to build here.
class MockLocalizer : public slam::ILocalizer {
public:
    struct Step {
        common::PoseEstimate pose;
        slam::LocalizerStats stats;
        common::SparseDepthFrame depth;
        bool have_depth = false;
    };

    // --- ILocalizer -------------------------------------------------------

    bool initialize(const slam::LocalizerConfig& config) override {
        last_config = config;
        ++initialize_calls;
        return initialize_succeeds;
    }

    common::PoseEstimate processFrame(const video::Frame& frame) override {
        last_frame_index = frame.index;
        ++frames_processed;
        if (script.empty()) return {};
        // Past the end, hold the last step. A test that scripts 10 poses and
        // pushes 12 frames is asking about the 10 it wrote, not about a cliff.
        last_step = std::min(next_step, script.size() - 1);
        if (next_step + 1 < script.size()) ++next_step;
        return script[last_step].pose;
    }

    common::TrackingState state() const override { return current().pose.state; }

    slam::LocalizerStats stats() const override { return current().stats; }

    bool trackedDepthSamples(common::SparseDepthFrame& out) const override {
        const Step& step = current();
        if (!step.have_depth || step.depth.samples.empty()) return false;
        out = step.depth;
        return true;
    }

    bool saveMap() override {
        ++save_map_calls;
        return save_map_succeeds;
    }

    bool saveTrajectory(const std::string& path) override {
        saved_trajectory_path = path;
        return save_trajectory_succeeds;
    }

    void shutdown() override { ++shutdown_calls; }

    // --- scripting --------------------------------------------------------

    // A well-tracked pose inside the pre-built map: the normal case.
    Step& appendTracked(const common::Vector3& position,
                         const common::Quaternion& orientation = {},
                         float confidence = 0.8f) {
        Step step;
        step.pose.pose.position = position;
        step.pose.pose.orientation = orientation;
        step.pose.state = common::TrackingState::Ok;
        step.pose.confidence = confidence;
        step.stats.in_prebuilt_map = true;
        step.stats.tracked_map_points = static_cast<int>(confidence * 100.0f);
        step.stats.detected_keypoints = 1500;
        step.stats.map_keyframes = map_keyframes_;
        step.stats.frames_processed = script.size() + 1;
        step.stats.frames_tracked = script.size() + 1;
        script.push_back(std::move(step));
        return script.back();
    }

    // Tracking gone. The pose carried here is meaningless by contract, and
    // every consumer must ignore it rather than fly to it.
    void appendLost(int frames = 1,
                     common::TrackingState state = common::TrackingState::Lost) {
        for (int i = 0; i < frames; ++i) {
            Step step;
            step.pose.state = state;
            step.pose.confidence = 0.0f;
            step.stats.in_prebuilt_map = true;
            step.stats.map_keyframes = map_keyframes_;
            script.push_back(std::move(step));
        }
    }

    // Tracking its own throwaway map, not yet merged with the pre-built one.
    // Poses are perfectly valid and mean nothing, which is exactly why
    // in_prebuilt_map rather than TrackingState::Ok is the flag that gates a
    // mission start.
    void appendUnmatched(int frames = 1) {
        for (int i = 0; i < frames; ++i) {
            Step step;
            step.pose.state = common::TrackingState::Recovering;
            step.pose.confidence = 0.5f;
            step.stats.in_prebuilt_map = false;
            step.stats.map_keyframes = 12;
            script.push_back(std::move(step));
        }
    }

    // A loop closure: the map frame shifts, so the same physical place now
    // reports a different position, and the keyframe count jumps. Anything
    // holding geometry in map coordinates has to notice and discard it.
    void appendMapFrameJump(const common::Vector3& shift, int keyframeJump = 400) {
        map_origin_shift_.x += shift.x;
        map_origin_shift_.y += shift.y;
        map_origin_shift_.z += shift.z;
        map_keyframes_ += keyframeJump;
        Step& step = appendTracked(map_origin_shift_);
        step.stats.map_id = ++map_id_;
    }

    // Attaches synthetic sparse depth to the most recently appended step:
    // `count` samples spread over the image, all on a fronto-parallel plane
    // at `depth_m`, optionally with the network-side noise a real fit sees.
    void attachPlanarDepth(const common::CameraIntrinsics& intrinsics, float depth_m,
                            int count = 60, float noise_m = 0.0f) {
        if (script.empty()) return;
        common::SparseDepthFrame& depth = script.back().depth;
        depth.samples.clear();
        depth.image_width = intrinsics.width;
        depth.image_height = intrinsics.height;
        depth.frame_index = script.size() - 1;

        // A deterministic spread: tests must not be flaky, and a fixed lattice
        // covers the image more evenly than a small random sample anyway.
        const int columns = std::max(1, static_cast<int>(std::sqrt(count)));
        const int rows = std::max(1, (count + columns - 1) / columns);
        std::uint32_t seed = 0x9e3779b9u;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < columns && static_cast<int>(depth.samples.size()) < count; ++c) {
                seed = seed * 1664525u + 1013904223u;
                const float jitter = noise_m * ((seed >> 8 & 0xffff) / 32768.0f - 1.0f);
                common::DepthSample sample;
                sample.u = intrinsics.width * (c + 0.5f) / columns;
                sample.v = intrinsics.height * (r + 0.5f) / rows;
                sample.depth_m = depth_m + jitter;
                sample.weight = 1.0f;
                depth.samples.push_back(sample);
            }
        }
        script.back().have_depth = true;
    }

    void reset() {
        next_step = 0;
        last_step = 0;
    }

    // The step most recently returned by processFrame() - NOT the next one.
    //
    // stats() and trackedDepthSamples() describe the frame just processed,
    // which is how the real localizer behaves and how every caller uses them
    // (pose and stats are read together, about the same frame).
    const Step& current() const {
        static const Step kEmpty;
        if (script.empty()) return kEmpty;
        return script[std::min(last_step, script.size() - 1)];
    }

    // --- scripted behaviour ----------------------------------------------
    std::vector<Step> script;
    bool initialize_succeeds = true;
    bool save_map_succeeds = true;
    bool save_trajectory_succeeds = true;

    // --- recorded state ---------------------------------------------------
    std::size_t next_step = 0;
    std::size_t last_step = 0;
    slam::LocalizerConfig last_config{};
    std::uint64_t last_frame_index = 0;
    std::uint64_t frames_processed = 0;
    int initialize_calls = 0;
    int save_map_calls = 0;
    int shutdown_calls = 0;
    std::string saved_trajectory_path;

private:
    common::Vector3 map_origin_shift_{};
    int map_keyframes_ = 600;
    unsigned long map_id_ = 0;
};

} // namespace mocks
