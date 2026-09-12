#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

#include "common/types.hpp"

namespace video {

// One decoded camera image on its way from a source (drone or file) to SLAM.
//
// Ownership contract, fixed here on purpose because retrofitting it later is
// painful: `image` is treated as IMMUTABLE once a Frame has been handed to
// anyone else. cv::Mat is a refcounted handle, so copying a Frame is cheap
// and shares pixels - which is only safe because nobody writes through a
// Frame they did not just create. A consumer that needs to draw on the image
// (e.g. an overlay) must call `image.clone()` first.
struct Frame {
    cv::Mat image;               // BGR8 (3-channel) as produced by every source
    common::Timestamp stamp{};   // steady_clock, for latency/watchdog logic

    // Seconds since the start of this stream. This - not `stamp` - is what
    // goes to ORB-SLAM3: it must be monotonic and consistent between a
    // recorded video and the live flight, which wall/steady clock time is
    // not.
    double seconds = 0.0;

    std::uint64_t index = 0;     // frame counter since source open
};

} // namespace video
