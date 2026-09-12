#include "slam/orb_slam3_localizer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <unistd.h>

#include <opencv2/core/persistence.hpp>

#include "common/logging.hpp"
#include "slam/map_metadata.hpp"

#include <System.h>

namespace slam {
namespace {

namespace fs = std::filesystem;

// ORB-SLAM3 builds its atlas path as literally `"./" + <key> + ".osa"` (see
// System::SaveAtlas / System::LoadAtlas). An absolute path in the key
// therefore does NOT work - `"./" + "/home/lin/map"` is the relative path
// "./home/lin/map" - and the ".osa" suffix must not be included or the file
// becomes "map.osa.osa". So convert whatever the caller gave us into a
// cwd-relative, extension-free stem.
std::string atlasKeyForMapPath(const std::string& mapPath) {
    fs::path absolute = fs::absolute(mapPath);
    absolute.replace_extension();

    std::error_code ec;
    const fs::path relative = fs::relative(absolute, fs::current_path(), ec);
    if (ec || relative.empty()) {
        common::logWarn("OrbSlam3Localizer",
                         "cannot express map path relative to the working directory; "
                         "ORB-SLAM3 will resolve it against the working directory");
        return absolute.string();
    }
    return relative.string();
}

std::string utcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&seconds, &utc);
    std::ostringstream out;
    out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return out.str();
}

// ORB-SLAM3 takes its whole configuration from a single YAML file, including
// which map to load or save. Rather than making users maintain two nearly
// identical camera configs (one for mapping, one for localizing), the one
// hand-written camera config is copied to a temporary file with the atlas
// keys appended for this session.
bool writeSessionSettings(const LocalizerConfig& config, const fs::path& outputPath) {
    std::ifstream cameraConfig(config.camera_config_path);
    if (!cameraConfig) {
        common::logError("OrbSlam3Localizer",
                          "cannot read camera config: " + config.camera_config_path);
        return false;
    }

    std::ofstream out(outputPath);
    if (!out) {
        common::logError("OrbSlam3Localizer",
                          "cannot write session settings: " + outputPath.string());
        return false;
    }

    out << cameraConfig.rdbuf();
    out << "\n# --- appended per session by slam::OrbSlam3Localizer ---\n";

    const std::string atlasKey = atlasKeyForMapPath(config.map_path);
    if (config.mode == LocalizerMode::Mapping) {
        // Both keys together when extending: ORB-SLAM3 loads the existing
        // atlas, maps the new session on top, and writes the whole thing out
        // on shutdown.
        if (!config.extend_map_path.empty()) {
            out << "System.LoadAtlasFromFile: \"" << atlasKeyForMapPath(config.extend_map_path)
                << "\"\n";
        }
        out << "System.SaveAtlasToFile: \"" << atlasKey << "\"\n";
    } else {
        out << "System.LoadAtlasFromFile: \"" << atlasKey << "\"\n";
    }
    return out.good();
}

common::TrackingState toTrackingState(int orbSlamState) {
    // ORB_SLAM3::Tracking::eTrackingState
    switch (orbSlamState) {
        case 2:  // OK
        case 5:  // OK_KLT
            return common::TrackingState::Ok;
        case 3:  // RECENTLY_LOST - pose is extrapolated, not observed
            return common::TrackingState::Recovering;
        case 4:  // LOST
            return common::TrackingState::Lost;
        default:  // SYSTEM_NOT_READY, NO_IMAGES_YET, NOT_INITIALIZED
            return common::TrackingState::NotInitialized;
    }
}

}  // namespace

struct OrbSlam3Localizer::Impl {
    LocalizerConfig config;
    std::unique_ptr<ORB_SLAM3::System> system;
    fs::path session_settings_path;
    MapMetadata metadata;

    std::atomic<int> raw_state{-1};
    LocalizerStats stats;
    bool shut_down = false;

    // Merge tracking (localization mode) - see updateMapIdentity().
    int extending_keyframes = 0;
    bool have_keyframe_baseline = false;
    int previous_keyframes = 0;
    bool localization_mode_frozen = false;
    int tracked_since_merge = 0;
};

OrbSlam3Localizer::OrbSlam3Localizer() : impl_(std::make_unique<Impl>()) {}

OrbSlam3Localizer::~OrbSlam3Localizer() { shutdown(); }

bool OrbSlam3Localizer::initialize(const LocalizerConfig& config) {
    impl_->config = config;

    // Upstream reacts to a bad vocabulary path or an unloadable map by
    // calling exit(-1) from inside the System constructor, which would take
    // a flight app down with no chance to land. Everything it would abort on
    // is therefore checked here first.
    if (!fs::exists(config.vocabulary_path)) {
        common::logError("OrbSlam3Localizer",
                          "ORB vocabulary not found: " + config.vocabulary_path +
                              " (run scripts/fetch_vocabulary.sh)");
        return false;
    }
    if (!fs::exists(config.camera_config_path)) {
        common::logError("OrbSlam3Localizer",
                          "camera config not found: " + config.camera_config_path);
        return false;
    }

    if (config.mode == LocalizerMode::Localization) {
        if (!fs::exists(config.map_path)) {
            common::logError("OrbSlam3Localizer", "map not found: " + config.map_path +
                                                       " (build one with apps/map_builder)");
            return false;
        }
        if (loadMapMetadata(config.map_path, impl_->metadata)) {
            if (!impl_->metadata.scale_calibrated) {
                common::logWarn("OrbSlam3Localizer",
                                 "map scale is uncalibrated: poses are in arbitrary SLAM units, "
                                 "not metres");
            }
            // An explicit override in the config wins; otherwise take the
            // scale the map was calibrated with.
            if (impl_->config.scale_metres_per_unit == 1.0) {
                impl_->config.scale_metres_per_unit = impl_->metadata.scale_metres_per_unit;
            }
        }
    } else {
        if (!config.extend_map_path.empty()) {
            if (!fs::exists(config.extend_map_path)) {
                common::logError("OrbSlam3Localizer",
                                  "map to extend not found: " + config.extend_map_path);
                return false;
            }
            if (fs::absolute(config.extend_map_path) == fs::absolute(config.map_path)) {
                common::logError("OrbSlam3Localizer",
                                  "--extend and the output map must be different files: "
                                  "ORB-SLAM3 deletes the output before writing it, so a failed "
                                  "save would destroy the map being extended");
                return false;
            }
            // Carry the original's provenance forward. The scale especially:
            // a merged session inherits the old map's scale, so re-measuring
            // it would be wasted work.
            if (loadMapMetadata(config.extend_map_path, impl_->metadata)) {
                impl_->extending_keyframes = impl_->metadata.keyframes;
                common::logInfo("OrbSlam3Localizer",
                                 "extending " + config.extend_map_path + " (" +
                                     std::to_string(impl_->extending_keyframes) + " keyframes)");
            }
        }
        std::error_code ec;
        fs::create_directories(fs::absolute(config.map_path).parent_path(), ec);
    }

    std::error_code ec;
    const fs::path sessionDir = fs::temp_directory_path(ec) / "tello_slam";
    fs::create_directories(sessionDir, ec);
    impl_->session_settings_path =
        sessionDir / ("session_" + std::to_string(::getpid()) + ".yaml");
    if (!writeSessionSettings(impl_->config, impl_->session_settings_path)) {
        return false;
    }

    const bool mapping = config.mode == LocalizerMode::Mapping;
    common::logInfo("OrbSlam3Localizer",
                     std::string("starting ORB-SLAM3 in ") + (mapping ? "MAPPING" : "LOCALIZATION") +
                         " mode, map=" + config.map_path);

    impl_->system = std::make_unique<ORB_SLAM3::System>(
        config.vocabulary_path, impl_->session_settings_path.string(),
        ORB_SLAM3::System::MONOCULAR, config.enable_viewer);

    // NOTE: ActivateLocalizationMode() is deliberately NOT called here, and
    // by default is never called at all.
    //
    // ORB-SLAM3 cannot relocalize directly into a loaded map: on load it
    // calls Atlas::CreateNewMap() and tracks in that new empty map, and
    // Relocalization() only ever searches keyframes belonging to the ACTIVE
    // map (KeyFrameDatabase::DetectRelocalizationCandidates filters on it).
    // The only route into the pre-built map is LoopClosing's place
    // recognition merging the session map into it - and that thread does
    // nothing while localization mode is on.
    //
    // So mapping stays enabled, the merge happens, and the map is left
    // unfrozen unless LocalizerConfig::freeze_after_tracked_frames says
    // otherwise. The map file on disk is never written either way.

    impl_->metadata.camera_config = config.camera_config_path;
    impl_->metadata.vocabulary = config.vocabulary_path;
    if (!config.extend_map_path.empty() && !impl_->metadata.source.empty()) {
        impl_->metadata.source += " + " + config.source_description;
    } else {
        impl_->metadata.source = config.source_description;
    }
    impl_->metadata.scale_metres_per_unit = impl_->config.scale_metres_per_unit;
    return true;
}

// Reads which map the tracked points belong to, and turns the moment the
// session map is merged into the pre-built one into a usable signal.
//
// Detecting that merge is harder than it should be. The obvious test - "has
// the active map's id changed?" - does not work, because ORB_SLAM3::Map::nNextId
// is a plain static counter that deserialization does not restore: a loaded
// map keeps its serialized id 0, and the scratch map created on top of it is
// handed id 0 as well. The ids are simply ambiguous.
//
// What is unambiguous is the size of the jump. Local mapping adds keyframes
// one at a time, so between two consecutive frames the count moves by at most
// one; a merge splices in the entire pre-built map at once, dozens of
// keyframes in a single step. That step is the signal.
void OrbSlam3Localizer::updateMapIdentity(
    const std::vector<ORB_SLAM3::MapPoint*>& trackedPoints) {
    ORB_SLAM3::Map* map = nullptr;
    for (ORB_SLAM3::MapPoint* point : trackedPoints) {
        if (point) {
            map = point->GetMap();
            if (map) break;
        }
    }
    if (!map) return;

    const int keyframes = static_cast<int>(map->KeyFramesInMap());
    impl_->stats.map_id = map->GetId();
    impl_->stats.map_keyframes = keyframes;

    if (impl_->config.mode == LocalizerMode::Mapping) {
        impl_->stats.in_prebuilt_map = true;
        impl_->previous_keyframes = keyframes;
        return;
    }

    if (!impl_->have_keyframe_baseline) {
        impl_->have_keyframe_baseline = true;
        impl_->previous_keyframes = keyframes;
        return;
    }

    if (impl_->stats.in_prebuilt_map) {
        impl_->previous_keyframes = keyframes;
        return;
    }

    // Scale the threshold to the map when its size is known, with a floor
    // that still cannot be reached one keyframe at a time.
    const int prebuilt = impl_->metadata.keyframes;
    const int threshold = prebuilt > 0 ? std::max(4, prebuilt / 3) : 4;
    const int jump = keyframes - impl_->previous_keyframes;
    impl_->previous_keyframes = keyframes;

    if (jump < threshold) return;

    impl_->stats.in_prebuilt_map = true;
    common::logInfo("OrbSlam3Localizer",
                     "merged into the pre-built map (" + std::to_string(keyframes) +
                         " keyframes, up from " + std::to_string(keyframes - jump) +
                         ") - poses are now in the map's own frame");
}

// Switches ORB-SLAM3 into localization-only mode, if the caller asked for it.
//
// Off by default, and the reasoning is in LocalizerConfig - briefly: freezing
// measured at 0.3-1.0% sustained tracking against a real map, versus 99.4%
// without. Kept as a knob because on a very large map the CPU saved by
// stopping local mapping may be worth the fragility, but it is not something
// to turn on without measuring the tracking rate first.
void OrbSlam3Localizer::maybeFreezeMap() {
    if (impl_->localization_mode_frozen) return;
    if (impl_->config.mode != LocalizerMode::Localization) return;
    if (!impl_->stats.in_prebuilt_map) return;
    if (impl_->config.freeze_after_tracked_frames <= 0) return;
    if (impl_->tracked_since_merge < impl_->config.freeze_after_tracked_frames) return;

    impl_->system->ActivateLocalizationMode();
    impl_->localization_mode_frozen = true;
    common::logInfo("OrbSlam3Localizer",
                     "map frozen after " + std::to_string(impl_->tracked_since_merge) +
                         " well-tracked frames - localization only from here");
}

common::PoseEstimate OrbSlam3Localizer::processFrame(const video::Frame& frame) {
    common::PoseEstimate estimate;
    estimate.pose.stamp = frame.stamp;

    if (!impl_->system || impl_->shut_down || frame.image.empty()) {
        estimate.state = common::TrackingState::NotInitialized;
        return estimate;
    }

    const Sophus::SE3f Tcw = impl_->system->TrackMonocular(frame.image, frame.seconds);

    const int rawState = impl_->system->GetTrackingState();
    impl_->raw_state.store(rawState, std::memory_order_relaxed);
    estimate.state = toTrackingState(rawState);

    if (impl_->stats.frames_processed == 0) {
        impl_->metadata.image_width = frame.image.cols;
        impl_->metadata.image_height = frame.image.rows;
    }
    ++impl_->stats.frames_processed;

    const auto trackedPoints = impl_->system->GetTrackedMapPoints();
    const int tracked = static_cast<int>(
        std::count_if(trackedPoints.begin(), trackedPoints.end(),
                      [](const ORB_SLAM3::MapPoint* point) { return point != nullptr; }));
    impl_->stats.tracked_map_points = tracked;
    impl_->stats.detected_keypoints = static_cast<int>(trackedPoints.size());

    updateMapIdentity(trackedPoints);

    if (estimate.state == common::TrackingState::Ok &&
        impl_->config.mode == LocalizerMode::Localization && !impl_->stats.in_prebuilt_map) {
        // Tracking really is fine - but in the session's own scratch map, so
        // the pose is not expressed in the frame the caller asked about.
        // Reporting Ok here is how a mission would end up flying to waypoints
        // in a coordinate system that has nothing to do with them.
        estimate.state = common::TrackingState::Recovering;
    }

    if (estimate.state != common::TrackingState::Ok) {
        impl_->tracked_since_merge = 0;
        // TrackMonocular still returns a pose when tracking has failed - the
        // last known or motion-model-extrapolated one. Publishing it would
        // let the controller chase a stale position, so it is dropped here
        // and the caller sees only the state.
        return estimate;
    }
    ++impl_->stats.frames_tracked;

    // Consecutive, not cumulative: a relapse into LOST means the merge has
    // not settled, so the count restarts.
    ++impl_->tracked_since_merge;
    maybeFreezeMap();

    // ORB-SLAM3 returns Tcw (world -> camera). The controller wants the
    // camera's position in the map, i.e. the inverse.
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f translation = Twc.translation();
    const Eigen::Quaternionf rotation = Twc.unit_quaternion();

    const auto scale = static_cast<float>(impl_->config.scale_metres_per_unit);
    estimate.pose.position = {translation.x() * scale, translation.y() * scale,
                               translation.z() * scale};
    estimate.pose.orientation = {rotation.w(), rotation.x(), rotation.y(), rotation.z()};

    // A frame matched against very few map points is geometrically weak even
    // though tracking nominally succeeded. 100 inliers is a comfortably
    // well-constrained frame for ORB-SLAM3's monocular pipeline.
    estimate.confidence = std::clamp(static_cast<float>(tracked) / 100.0f, 0.0f, 1.0f);
    return estimate;
}

common::TrackingState OrbSlam3Localizer::state() const {
    return toTrackingState(impl_->raw_state.load(std::memory_order_relaxed));
}

LocalizerStats OrbSlam3Localizer::stats() const { return impl_->stats; }

bool OrbSlam3Localizer::saveTrajectory(const std::string& path) {
    if (!impl_->system || impl_->shut_down) return false;

    std::error_code ec;
    fs::create_directories(fs::absolute(path).parent_path(), ec);
    impl_->system->SaveKeyFrameTrajectoryTUM(path);

    if (!fs::exists(path)) {
        common::logError("OrbSlam3Localizer", "trajectory was not written to " + path);
        return false;
    }
    impl_->metadata.trajectory_file = path;
    common::logInfo("OrbSlam3Localizer", "wrote keyframe trajectory to " + path);
    return true;
}

bool OrbSlam3Localizer::saveMap() {
    if (impl_->config.mode != LocalizerMode::Mapping) {
        common::logError("OrbSlam3Localizer", "saveMap() is only valid in mapping mode");
        return false;
    }
    if (!impl_->system || impl_->shut_down) return false;

    // ORB-SLAM3 serializes its atlas from inside Shutdown() and nowhere else,
    // so saving necessarily ends the session. Hence the documented contract
    // that no frame may be processed after saveMap().
    common::logInfo("OrbSlam3Localizer", "shutting down to serialize the map...");
    shutdown();

    if (!fs::exists(impl_->config.map_path)) {
        common::logError("OrbSlam3Localizer",
                          "ORB-SLAM3 did not write the map to " + impl_->config.map_path +
                              " - was the map ever initialized? (tracking must reach OK at least "
                              "once for a map to exist)");
        return false;
    }

    if (impl_->extending_keyframes > 0) {
        // Merged sessions land in one map, so the keyframe count grows past
        // the original. A count that did NOT grow means place recognition
        // never matched, and the atlas now holds a second, disconnected map -
        // which no waypoint authored against the original can be flown in.
        if (impl_->stats.map_keyframes < impl_->extending_keyframes) {
            common::logWarn("OrbSlam3Localizer",
                             "the new session did NOT merge into the map being extended (" +
                                 std::to_string(impl_->stats.map_keyframes) + " keyframes vs " +
                                 std::to_string(impl_->extending_keyframes) +
                                 " originally). The saved atlas contains two disconnected maps, "
                                 "and the existing route cannot be flown in the new one. Record "
                                 "footage that overlaps the original route more closely.");
        } else {
            common::logInfo("OrbSlam3Localizer",
                             "merged into the extended map: " +
                                 std::to_string(impl_->stats.map_keyframes) + " keyframes, up "
                                 "from " + std::to_string(impl_->extending_keyframes));
        }
    }

    impl_->metadata.created_utc = utcTimestamp();
    impl_->metadata.frames_processed = impl_->stats.frames_processed;
    impl_->metadata.frames_tracked = impl_->stats.frames_tracked;
    // Never shrink the recorded keyframe count when extending. The merge
    // detector sizes its threshold from this number, so recording the small
    // unmerged session here would let a later flight mistake "merged into the
    // 30-keyframe offcut" for "merged into the 623-keyframe route map", and
    // fly waypoints from one coordinate frame while localized in another.
    impl_->metadata.keyframes =
        std::max(impl_->stats.map_keyframes, impl_->extending_keyframes);
    impl_->metadata.scale_metres_per_unit = impl_->config.scale_metres_per_unit;
    saveMapMetadata(impl_->config.map_path, impl_->metadata);

    common::logInfo("OrbSlam3Localizer", "map saved to " + impl_->config.map_path);
    return true;
}

void OrbSlam3Localizer::shutdown() {
    if (!impl_->system || impl_->shut_down) return;
    impl_->shut_down = true;

    impl_->system->Shutdown();
    impl_->system.reset();

    std::error_code ec;
    if (!impl_->session_settings_path.empty()) {
        fs::remove(impl_->session_settings_path, ec);
    }
}

} // namespace slam
