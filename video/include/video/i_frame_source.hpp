#pragma once

#include <optional>
#include <string>

#include "video/frame.hpp"

namespace video {

// Where decoded frames come from. Two implementations exist and they are
// deliberately interchangeable:
//
//   TelloVideoSource  - live H264 over UDP 11111 from the drone
//   VideoFileSource   - a recorded video file
//
// That interchangeability is the whole point of this abstraction for this
// project: the map is built offline from a file and used online from the
// drone, and both paths must feed byte-identical Frames into the same SLAM
// code, or a map built offline will not relocalize online.
class IFrameSource {
public:
    virtual ~IFrameSource() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    // Returns the next frame, waiting up to timeoutMs for one. nullopt means
    // "nothing available yet" for a live source, and "end of stream" for a
    // file source - distinguish the two with isOpen().
    virtual std::optional<Frame> nextFrame(int timeoutMs) = 0;

    // Nominal frame rate, used to size timeouts and to pace playback. 0 if
    // unknown.
    virtual double nominalFps() const = 0;

    // One-line health summary for diagnostics, or empty if the source has
    // nothing useful to say. Exists so that an app whose video has stalled
    // can tell the user WHY on screen, instead of just sitting there.
    virtual std::string healthSummary() const { return {}; }
};

} // namespace video
