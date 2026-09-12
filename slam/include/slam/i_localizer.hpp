#pragma once

#include <string>

#include "common/types.hpp"
#include "video/frame.hpp"

namespace slam {

enum class LocalizerMode {
    // Build a new map from scratch. Local mapping and loop closing run, the
    // map grows, and the result is written out on saveMap().
    Mapping,
    // Track against an existing map without ever modifying it. This is the
    // mode used in flight.
    Localization,
};

struct LocalizerConfig {
    LocalizerMode mode = LocalizerMode::Localization;

    // ORB vocabulary (ORBvoc.txt). The SAME file must be used for mapping and
    // for localization - ORB-SLAM3 stores a checksum of it in the map and
    // refuses a map built with a different one.
    std::string vocabulary_path;

    // Camera intrinsics + ORB tuning, in ORB-SLAM3's own YAML schema. See
    // config/tello_camera.yaml.
    std::string camera_config_path;

    // Mapping: where the map is written by saveMap().
    // Localization: the map to load. Must already exist.
    std::string map_path;

    // MAPPING MODE ONLY: start from this existing map instead of from
    // nothing, so a new session is added to it rather than replacing it.
    //
    // This is how a map survives a change in conditions. A map built at
    // midday stops matching the same room under evening lighting - measured
    // on this project's own footage, contrast fell 30% and ORB feature
    // counts 16%, and relocalization went from 96% of frames to none at all.
    // Rebuilding from scratch would work, but it throws away the scale
    // calibration and every waypoint authored against the old frame.
    // Extending keeps both: if ORB-SLAM3's place recognition matches the new
    // session to the old map, the two are merged into one map in the old
    // coordinate frame.
    std::string extend_map_path;

    // Switch ORB-SLAM3 into localization-only mode after this many
    // consecutive well-tracked frames inside the pre-built map.
    //
    // DEFAULT 0 - NEVER. Freezing sounds obviously right ("don't let the
    // flight modify the validated map") and measures catastrophically badly.
    // Replaying a 623-keyframe office map against the footage it was built
    // from, sustained tracking was:
    //
    //     freeze immediately on merge ...........    32 / 11848 frames  (0.3%)
    //     freeze after 90 tracked frames ........   118 / 11848 frames  (1.0%)
    //     never freeze ..........................  11777 / 11848 frames (99.4%)
    //
    // The delay was an attempt to dodge a race - LoopClosing::MergeLocal()
    // ends by calling LocalMapping::Release(), undoing the stop - but the
    // numbers say the race was not the real problem. ORB-SLAM3's
    // localization-only mode cannot create map points, so the moment the
    // camera looks anywhere the map does not already cover well it drops to
    // RECENTLY_LOST and thrashes between relocalization attempts.
    //
    // Not freezing costs nothing in safety: the map ON DISK is never written
    // in localization mode (the session config carries only
    // System.LoadAtlasFromFile, so System::SaveAtlas is never reached).
    // Only the in-memory copy grows, and that growth is what keeps tracking
    // alive. The real cost is that loop closure stays active, so a large
    // closure can shift the map frame mid-flight - see the README.
    int freeze_after_tracked_frames = 0;

    // Recorded in the map's metadata so a map can be traced back to the
    // footage it came from, e.g. "recordings/office_route.h264".
    std::string source_description;

    // Pangolin map/frame viewer. Useful while building a map, and worth
    // turning off in flight to save CPU.
    bool enable_viewer = true;

    // Scale factor converting SLAM units to metres. Monocular SLAM has no
    // absolute scale, so this must be measured (see tools/calibrate_scale)
    // and is normally loaded from the map's metadata sidecar rather than set
    // by hand. Poses returned by processFrame() are already multiplied by it,
    // so downstream control code always works in metres.
    double scale_metres_per_unit = 1.0;
};

// What a localizer knows about the frame it just processed. Reported
// alongside the pose so control code can decide whether to trust it.
struct LocalizerStats {
    int tracked_map_points = 0;   // map points matched in this frame
    int detected_keypoints = 0;   // ORB features extracted from this frame
    std::uint64_t frames_processed = 0;
    std::uint64_t frames_tracked = 0;   // frames where state was Ok

    // Keyframes in the map currently being tracked, and its ORB-SLAM3 id.
    int map_keyframes = 0;
    unsigned long map_id = 0;

    // LOCALIZATION MODE ONLY, and the single most important flag here.
    //
    // Loading a map does not put ORB-SLAM3 *into* it. It opens a fresh, empty
    // map, tracks in that, and only once its place recognition matches the
    // pre-built map does it merge the two and adopt the pre-built map's
    // coordinate frame. Until that happens the poses are perfectly valid -
    // in a throwaway frame where the mission's waypoints mean nothing.
    //
    // So this, not TrackingState::Ok, is what "we know where we are in the
    // map" means. processFrame() will not report Ok until it is true.
    bool in_prebuilt_map = false;
};

// Visual localization, with ORB-SLAM3 as the only implementation today.
//
// This interface exists specifically to keep ORB-SLAM3's types (cv::Mat
// poses, Sophus::SE3f, Atlas, ...) out of the rest of the project: nothing
// above this line includes an ORB-SLAM3 header, so replacing it is a change
// confined to slam/.
class ILocalizer {
public:
    virtual ~ILocalizer() = default;

    // Loads the vocabulary and (in Localization mode) the map. Returns false
    // rather than aborting on a bad path - see the note in
    // orb_slam3_localizer.cpp about upstream's exit(-1) habit.
    virtual bool initialize(const LocalizerConfig& config) = 0;

    // Feeds one image and returns the drone's estimated pose IN THE MAP
    // FRAME (world-from-camera), with the translation already scaled to
    // metres. When state != Ok the pose is meaningless and must not be used
    // for control.
    virtual common::PoseEstimate processFrame(const video::Frame& frame) = 0;

    virtual common::TrackingState state() const = 0;
    virtual LocalizerStats stats() const = 0;

    // Mapping mode only. Writes the map to LocalizerConfig::map_path plus a
    // metadata sidecar. Ends the session: ORB-SLAM3 serializes its atlas from
    // inside Shutdown(), so no frame may be processed afterwards.
    virtual bool saveMap() = 0;

    // Writes the keyframe trajectory in TUM format
    // (timestamp tx ty tz qx qy qz qw). This is how a flight route is
    // authored: build a map by flying the route, then pick waypoints off the
    // trajectory it produced.
    virtual bool saveTrajectory(const std::string& path) = 0;

    virtual void shutdown() = 0;
};

} // namespace slam
