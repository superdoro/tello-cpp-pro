#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "common/types.hpp"

namespace control {

// A point on the route, expressed in the MAP frame (see frame_alignment.hpp)
// with distances in METRES - which is only meaningful once the map's scale
// has been calibrated.
struct Waypoint {
    common::Vector3 position;

    // Desired heading about the map's up axis, radians, same convention as
    // FrameAlignment::headingInMap. Ignored when hold_heading is false, which
    // lets a route say "get there, don't care which way you face".
    float heading_rad = 0.0f;
    bool hold_heading = false;

    float position_tolerance_m = 0.25f;
    float heading_tolerance_rad = 0.35f;  // ~20 degrees

    // How long the drone must stay inside tolerance before the waypoint
    // counts as reached. Non-zero stops a fast fly-through from being
    // mistaken for arrival.
    std::chrono::milliseconds dwell{300};

    // Give up on this waypoint (and abort the mission) after this long.
    std::chrono::milliseconds timeout{20000};

    std::string label;
};

enum class MissionStatus {
    Idle,       // loaded, not started
    Running,    // flying towards currentTarget()
    Holding,    // pose untrustworthy: hovering, waiting for tracking to recover
    Complete,   // every waypoint reached
    Aborted,    // safety limit hit; the app should land
};

struct GeofenceConfig {
    // Maximum distance, in metres, the drone may be from the map origin.
    // A bad pose estimate is the most likely cause of a runaway, and this is
    // the cheapest backstop against one.
    float max_radius_m = 15.0f;
    float max_altitude_m = 3.0f;
    float min_altitude_m = -1.0f;
};

struct MissionConfig {
    GeofenceConfig geofence;

    // Fly THROUGH intermediate waypoints instead of stopping at each one.
    //
    // Stopping at every waypoint is what makes a route feel like it stalls:
    // the proportional term shrinks as the error does, so the last 20cm of
    // each hop is crawled, held for the dwell time, and then accelerated out
    // of again. With a route sampled every 0.6-0.8m off a recorded
    // trajectory, that is a stutter every metre.
    //
    // In continuous mode the controller instead chases a point `lookahead_m`
    // further along the path, and the index advances as the drone passes
    // each waypoint. The final waypoint is still a real stop, so the mission
    // ends where it should.
    bool continuous = true;

    // How far ahead along the route to aim. Too small and the drone weaves
    // along the path; too large and it cuts corners badly. Roughly 1.5-2x the
    // waypoint spacing is a reasonable starting point.
    float lookahead_m = 1.2f;

    // Extra lookahead per metre/second of actual speed, and the ceiling it
    // may reach.
    //
    // A fixed lookahead cannot serve both speeds. What matters is how far
    // AHEAD IN TIME the drone is aiming: 1.2m is a comfortable 1.5 seconds at
    // walking pace and a twitchy 0.4 seconds at 3m/s, where it shows up as
    // weaving on straights and overshoot at corners. Scaling with measured
    // speed keeps that horizon roughly constant, so raising --max-speed does
    // not also require re-tuning the route.
    float lookahead_time_s = 0.5f;
    float lookahead_max_m = 3.0f;

    // --- Slowing down for bends -------------------------------------------
    //
    // Shortening the lookahead at a corner aims the drone correctly but does
    // nothing about its speed: the cruise term keeps pushing and momentum
    // carries it wide, which indoors means into the wall on the outside of
    // the turn. These three shape a braking profile instead.

    // How far ahead to start braking, expressed in seconds of travel. Scaled
    // by the measured speed, because the faster the drone is going the
    // earlier it has to begin.
    float corner_brake_time_s = 1.5f;

    // A bend at least this sharp gets the full slowdown; gentler ones get
    // proportionally less.
    float corner_full_slow_rad = 1.2f;  // ~70 degrees

    // Floor on the cruise term at the sharpest corners, as a fraction. Not
    // zero: the drone should still be moving through the turn, just slowly.
    float corner_min_cruise_scale = 0.25f;

    // Stop extending the lookahead once the route has turned this far from
    // the direction it set off in.
    //
    // Aiming a fixed distance ahead cuts corners, and a cut corner indoors is
    // a wall: the straight line from the drone to a point 1.2m along the path
    // does not have to stay inside the room. Truncating the lookahead at a
    // bend makes the drone fly to the corner and turn there, which is what
    // the recorded route did. Straights are unaffected.
    float lookahead_corner_limit_rad = 0.6f;  // ~35 degrees

    // How long tracking may stay lost before the mission aborts rather than
    // continuing to hover. Hovering blind is safe briefly and reckless
    // indefinitely.
    std::chrono::milliseconds max_tracking_loss{4000};

    // Poses below this confidence are treated as untracked. ORB-SLAM3 can
    // report OK on very few inliers, and those poses jump.
    float min_confidence = 0.15f;
};

struct Mission {
    std::vector<Waypoint> waypoints;
    MissionConfig config;
    std::string name;

    // The map this route was authored against. Flying a route against a
    // different map silently produces nonsense, so the app checks it.
    std::string map_path;
};

} // namespace control
