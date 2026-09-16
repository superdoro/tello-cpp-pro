#pragma once

#include <cstdint>
#include <vector>

#include "common/types.hpp"

namespace common {

// One pixel whose metric depth is known from geometry rather than from a
// network.
struct DepthSample {
    // RAW image pixel coordinates - the same pixels a depth model sees.
    float u = 0.0f;
    float v = 0.0f;

    // Depth along the camera's z axis, in METRES.
    float depth_m = 0.0f;

    // How much to trust this sample, in [0, 1]. Derived from how many
    // keyframes observed the underlying map point: a freshly triangulated
    // point is much noisier than one seen from ten viewpoints.
    float weight = 1.0f;
};

// The sparse metric depths available for one frame.
//
// Produced by slam/ (from the map points ORB-SLAM3 matched in that frame) and
// consumed by perception/ (to fit a monocular depth network's unitless output
// to metres). Neither module knows about the other; this struct is the whole
// contract.
struct SparseDepthFrame {
    std::vector<DepthSample> samples;

    Timestamp stamp{};
    std::uint64_t frame_index = 0;

    // Resolution the sample coordinates are expressed in, so a consumer that
    // works at a different resolution can scale correctly.
    int image_width = 0;
    int image_height = 0;

    bool empty() const { return samples.empty(); }
};

} // namespace common
