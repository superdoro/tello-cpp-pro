#pragma once

#include <memory>
#include <string>

#include "video/i_frame_source.hpp"

namespace video {

struct VideoFileConfig {
    std::string path;

    // Frames are normally delivered as fast as they decode, because offline
    // map building should not be limited to real time. Set true to pace
    // playback at the file's own frame rate, e.g. when replaying a session to
    // reproduce a live timing bug.
    bool realtime_pacing = false;

    // Use every Nth frame. Handy when a video was shot at 30fps but the
    // motion is slow enough that ORB-SLAM3 gains nothing from every frame.
    int frame_stride = 1;

    // If non-zero, frames are resized to this resolution before being
    // returned. This exists to enforce a hard requirement of the whole
    // build-offline / fly-online plan: camera intrinsics (fx, fy, cx, cy) are
    // expressed in pixels, so a map built from 1280x720 video CANNOT be used
    // to localize a 960x720 live stream unless the images are brought to the
    // same resolution the calibration describes.
    int target_width = 0;
    int target_height = 0;
};

// Reads frames from a recorded video file. This is the source used to
// pre-build the map: fly (or walk) the route once, record it, then build the
// map offline as many times as needed while tuning, with no battery cost.
class VideoFileSource : public IFrameSource {
public:
    explicit VideoFileSource(VideoFileConfig config);
    ~VideoFileSource() override;

    bool open() override;
    void close() override;
    bool isOpen() const override;
    std::optional<Frame> nextFrame(int timeoutMs) override;
    double nominalFps() const override;

    // Total frames reported by the container, or 0 if unknown. Used only for
    // progress reporting.
    std::uint64_t frameCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace video
