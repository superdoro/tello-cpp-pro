#pragma once

// Moved to common/. The frame conventions this file used to define are needed
// by perception/ as well as control/, and duplicating the camera-mount and
// map_up conventions across two layers is exactly how you get a system that
// avoids an obstacle into a wall. There is now one definition, in the layer
// both depend on.
//
// This header stays so that control/ code (and its tests) can keep saying
// control::FrameAlignment.
#include "common/frame_alignment.hpp"

namespace control {

using common::FrameAlignment;

using common::conjugate;
using common::distance;
using common::normalize;
using common::rotate;
using common::wrapAngle;

} // namespace control
