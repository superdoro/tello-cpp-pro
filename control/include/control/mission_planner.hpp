#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>
#include <string>

#include "control/frame_alignment.hpp"
#include "control/waypoint.hpp"

namespace control {

// Sequences waypoints and owns every reason to stop flying.
//
// The planner never talks to the drone. It consumes poses and reports what
// the app should do, so all of its safety logic is exercisable in unit tests
// with synthetic pose sequences instead of a real flight.
class MissionPlanner {
public:
    void loadMission(Mission mission);
    void start();

    // Picks the mission back up at the waypoint it was on, restarting that
    // waypoint's timers. start() would jump back to the first waypoint, which
    // after a manual takeover means flying the whole route again.
    void resume();

    void abort(const std::string& reason);

    // Feed every pose estimate here, tracked or not - the loss timer needs to
    // see the untracked ones too.
    void onPoseUpdate(const common::PoseEstimate& pose,
                       std::chrono::steady_clock::time_point now);

    MissionStatus status() const { return status_; }

    // What the controller should fly at right now. In continuous mode this is
    // an interpolated point ahead on the path, not the indexed waypoint.
    std::optional<Waypoint> currentTarget() const;
    std::size_t currentIndex() const { return index_; }
    std::size_t waypointCount() const { return route_.size(); }

    // Which pass over the route is being flown (1-based), how many there are,
    // and whether this one runs backwards.
    int currentPass() const { return pass_; }
    int totalPasses() const { return std::max(mission_.config.passes, 1); }
    bool passIsReversed() const { return pass_reversed_; }
    const std::string& abortReason() const { return abort_reason_; }

    // Smoothed ground speed in m/s, and the lookahead it is currently buying.
    float speed() const { return speed_mps_; }
    float lookahead() const { return effectiveLookahead(); }

    // How much of the cruise command the route ahead can take, in [0, 1].
    // 1 on a straight, falling towards corner_min_cruise_scale as a bend
    // approaches. The flight controller scales only its feedforward term by
    // this - error correction is never weakened.
    float cruiseScale() const { return cruise_scale_; }

    // Vector from the drone to the nearest point on the route, in MAP
    // coordinates. Zero when on the path.
    //
    // Pure pursuit converges to a path only asymptotically, and the longer
    // the lookahead the weaker that pull becomes: the same sideways offset
    // subtends a smaller angle to a carrot further away, so at speed the
    // drone runs wide out of a corner and drifts back only slowly. Measuring
    // the offset directly gives the controller a correction that does not
    // weaken as the lookahead grows.
    common::Vector3 pathOffset() const { return path_offset_; }
    const Mission& mission() const { return mission_; }

    void setAlignment(const FrameAlignment& alignment) { alignment_ = alignment; }

private:
    bool violatesGeofence(const common::Vector3& position) const;

    // Walks the route forward from the current index until `lookahead_m` of
    // path has been covered, interpolating within the segment it lands in.
    Waypoint lookaheadTarget(const common::Vector3& from) const;

    // Lookahead distance for the current speed.
    float effectiveLookahead() const;

    void updateSpeed(const common::Vector3& position,
                      std::chrono::steady_clock::time_point now);

    // Looks ahead over the current braking distance for the sharpest bend and
    // sets cruise_scale_ from how sharp it is and how close.
    void updateCruiseScale(const common::Vector3& from);

    // Nearest point on the route near the current index, for path_offset_.
    void updatePathOffset(const common::Vector3& position);

    // Advances past every waypoint the drone has reached or already flown
    // past. Returns true if the index moved.
    bool advancePassedWaypoints(const common::Vector3& position,
                                 std::chrono::steady_clock::time_point now);

    bool onFinalWaypoint() const { return index_ + 1 >= route_.size(); }

    // Builds route_ for the given pass, reversing the order (and optionally
    // the headings) when that pass runs backwards.
    void buildRouteForPass(int pass);

    // Called when the last waypoint of a pass is reached. Starts the next
    // pass, or completes the mission.
    void finishPass(std::chrono::steady_clock::time_point now);

    Mission mission_;
    FrameAlignment alignment_;
    MissionStatus status_ = MissionStatus::Idle;
    std::size_t index_ = 0;

    // The waypoint order actually being flown this pass. Reversal and
    // repetition are expressed by rebuilding this, so nothing downstream has
    // to know a route can run backwards.
    std::vector<Waypoint> route_;
    int pass_ = 1;
    bool pass_reversed_ = false;
    std::string abort_reason_;
    std::optional<Waypoint> current_target_;

    // Smoothed ground speed, used to size the lookahead. Raw frame-to-frame
    // speed off SLAM poses is far too noisy to steer with.
    float speed_mps_ = 0.0f;
    float cruise_scale_ = 1.0f;
    common::Vector3 path_offset_{};
    common::Vector3 last_position_{};
    std::chrono::steady_clock::time_point last_pose_at_{};
    bool have_last_pose_ = false;

    std::chrono::steady_clock::time_point waypoint_started_at_{};
    std::chrono::steady_clock::time_point inside_tolerance_since_{};
    bool inside_tolerance_ = false;
    std::chrono::steady_clock::time_point tracking_lost_since_{};
    bool tracking_lost_ = false;
};

} // namespace control
