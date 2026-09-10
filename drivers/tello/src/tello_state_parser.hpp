#pragma once

#include <string>

#include "common/types.hpp"

namespace drivers::tello::detail {

// Parses a raw Tello state line, e.g.
//   "pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:60;temph:63;tof:10;h:0;
//    bat:87;baro:47.65;time:0;agx:0.00;agy:0.00;agz:-1000.00;"
// Returns false if the line has no recognizable key:value fields.
bool parseTelloState(const std::string& line, common::DroneState& outState);

} // namespace drivers::tello::detail
