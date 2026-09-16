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

// How close this route lets the drone get to things it can see.
//
// Mission policy, not sensing policy: config/perception.yaml says HOW to sense,
// and this says what to do about it on THIS route. A tight indoor corridor and
// an open hall want different numbers with identical cameras.
struct AvoidanceConfig {
    // Off by default. Turning obstacle avoidance on is a deliberate act, and a
    // route authored before the feature existed must fly exactly as it did.
    bool enabled = false;

    // Each layer can be disabled independently, which is how the first flights
    // are done: slow and brake only, no steering.
    bool enable_slow = true;
    bool enable_steer = true;
    bool enable_brake = true;

    // Clearance bands, in metres. Below brake the drone holds; below steer it
    // also aims to one side; below slow it merely eases off.
    float brake_distance_m = 1.0f;
    float steer_distance_m = 2.2f;
    float slow_distance_m = 4.0f;

    // Half-width of the tube ahead that must stay clear: the airframe plus a
    // margin, widened with range to cover pose and depth error growing with
    // distance.
    float corridor_radius_m = 0.45f;
    float corridor_radius_growth = 0.15f;  // extra metres of radius per metre of range

    // Floor on the cruise multiplier. Not zero: crawling is still progress,
    // and the brake layer is what actually stops the drone.
    float min_cruise_scale = 0.15f;

    // How far aside the target may be pushed, in the BODY frame.
    //
    // Deliberately small. This camera sees 55 degrees horizontally, so an
    // escape aimed much further out than this is aimed into space the drone
    // has no current evidence about.
    float max_lateral_offset_m = 0.8f;
    float max_vertical_offset_m = 0.5f;

    // Slew limit on the offset. At 30 Hz this is 0.02 m/step, comfortably
    // under FlightControllerConfig::target_jump_reset_m, so a manoeuvre never
    // trips the PID's "this is a new setpoint" reset and never turns into a
    // one-frame stick slam.
    float offset_rate_m_per_s = 0.6f;

    // --- hysteresis -------------------------------------------------------
    // Leaving a band needs this much more clearance than entering it did.
    float exit_margin_m = 0.35f;
    // No state change may follow another within this long.
    std::chrono::milliseconds min_state_dwell{400};
    // Once a side has been chosen to escape towards, stay with it this long.
    // Re-deciding every frame makes the drone weave between the two sides of
    // a pillar and commit to neither.
    std::chrono::milliseconds commit_time{1500};

    // --- trusting the input ----------------------------------------------
    // Older than this and the picture describes where the obstacles were, not
    // where they are.
    std::chrono::milliseconds max_snapshot_age{500};
    // Grace period after takeoff before a missing snapshot counts as a fault.
    std::chrono::milliseconds startup_grace{5000};

    // Below min_confidence nothing but the degraded speed cap applies. Below
    // steer_min_confidence the drone may still slow and brake, but may not
    // choose a direction to fly in - a wrong scale that stops you early is a
    // nuisance, a wrong scale that steers you is a crash.
    float min_confidence = 0.25f;
    float steer_min_confidence = 0.60f;

    // Speed multiplier while degraded. Set to 1.0 for strict parity with
    // flight behaviour before this feature existed.
    float degraded_cruise_scale = 0.60f;

    // Abort the mission if perception is unhealthy. Default false: enabling
    // avoidance must never make a previously flyable route unflyable.
    bool require_perception = false;

    // Give up after this long held at a standstill by something that is not
    // moving out of the way.
    std::chrono::milliseconds blocked_abort_after{8000};
};

struct MissionConfig {
    GeofenceConfig geofence;
    AvoidanceConfig avoidance;

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
    // --- Flying the route more than once ----------------------------------

    // How many passes over the route to fly. 1 is a single run.
    int passes = 1;

    // Fly the route backwards.
    //
    // The waypoint ORDER is reversed but each waypoint's recorded heading is
    // kept, so the drone flies backwards with the camera still facing the way
    // it faced when the map was built. That is deliberate: ORB features are
    // viewpoint-dependent, and a corridor looks nothing like itself from the
    // other end, so turning around to face the direction of travel would
    // point the camera at views the map does not contain. Set
    // reverse_headings as well only if the space was mapped in both
    // directions.
    bool reverse = false;
    bool reverse_headings = false;

    // Alternate direction on each pass: out, back, out, back.
    //
    // For a route that does not return to where it started, this is the only
    // safe way to repeat it. Plain repetition would finish at the far end and
    // then set course straight for the first waypoint - a line that owes
    // nothing to the mapped route and can run through a wall.
    bool ping_pong = false;

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
