#pragma once

#include <string>

#include "control/frame_alignment.hpp"
#include "control/waypoint.hpp"

namespace control {

// Missions are stored as YAML (config/mission_*.yaml). Both functions log and
// return false on failure rather than throwing - a malformed route file must
// not crash a program that may have a drone in the air.
bool loadMission(const std::string& path, Mission& mission);
bool saveMission(const std::string& path, const Mission& mission);

struct TrajectorySamplingConfig {
    // Keep one waypoint per this much travelled distance, in metres.
    float spacing_m = 0.6f;
    // Copy the recorded heading into each waypoint, so the drone retraces the
    // route facing the way the camera did. Almost always what you want: the
    // map was built from those views, so relocalizing from them is easiest.
    bool hold_heading = true;
    float position_tolerance_m = 0.25f;
    // Applied to every sampled point. The map's scale must already be known.
    double scale_metres_per_unit = 1.0;

    // Trim the route. A recording almost always contains footage from before
    // the interesting part - walking to the start, hunting for a view the
    // mapper likes - and a route whose first waypoint is somewhere the drone
    // will not be at takeoff just stalls there until its timeout.
    int skip_first = 0;
    int max_waypoints = 0;  // 0 = no limit

    // The generated mission's geofence is derived from the route it contains,
    // plus this margin in metres, rather than from a fixed default.
    //
    // A fixed default cannot work: it has no idea how big the mapped space
    // is, so it either aborts a legitimate route the instant it starts (a
    // 15m default against a 46m route) or is so loose it protects nothing.
    // Bounding the actual route is both correct by construction and tighter
    // than any guess.
    float geofence_margin_m = 3.0f;
};

// Turns a TUM-format keyframe trajectory (as written by
// ILocalizer::saveTrajectory) into a route.
//
// This closes the loop on the whole workflow: record a video of the route,
// build a map from it, and take the flight plan from the very trajectory the
// mapping run reconstructed - so every waypoint is somewhere the drone is
// known to be able to localize from.
bool missionFromTrajectory(const std::string& trajectoryPath,
                            const TrajectorySamplingConfig& config,
                            const FrameAlignment& alignment,
                            Mission& mission);

} // namespace control
