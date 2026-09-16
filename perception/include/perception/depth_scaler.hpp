#pragma once

#include <opencv2/core.hpp>

#include "common/camera_intrinsics.hpp"
#include "common/sparse_depth.hpp"

namespace perception {

// The affine fit that turns a monocular network's output into metres.
//
// Networks of the MiDaS family emit INVERSE depth, and only up to an unknown
// scale and shift - the relationship is
//
//     1 / depth_metres = a * relative + b
//
// affine in inverse depth, NOT in depth. Fitting depth directly against the
// network's output is the classic mistake here: it looks plausible on a scene
// at one distance and falls apart the moment the scene has both near and far
// structure, which is every corridor.
struct DepthScale {
    float a = 0.0f;
    float b = 0.0f;

    // [0, 1]. Feeds ObstacleSnapshot::depth_scale_confidence, which gates
    // whether the avoidance layer is allowed to steer or may only slow down.
    float confidence = 0.0f;

    int inliers = 0;
    int samples_used = 0;
    float median_error_m = 0.0f;

    bool valid() const { return confidence > 0.0f; }

    // Metres for one network output value. Returns infinity where the fit
    // puts the point at or behind the camera, which is the honest answer:
    // "this pixel is further than this fit can say".
    float metres(float relative) const {
        const float inverse = a * relative + b;
        return inverse > 1e-4f ? 1.0f / inverse : std::numeric_limits<float>::infinity();
    }
};

struct DepthScalerConfig {
    // Below this there is not enough geometry to fit two parameters robustly.
    int min_samples = 12;

    // RANSAC inlier test, in metres of depth error. Generous, because the
    // sparse depths are themselves triangulated from a monocular map.
    float inlier_tolerance_m = 0.25f;
    // ...or this fraction of the sample's own depth, whichever is larger. A
    // fixed tolerance is far too strict at 6m and far too loose at 0.5m.
    float inlier_tolerance_fraction = 0.12f;

    int ransac_iterations = 96;

    // Samples outside this range are dropped before fitting: closer than this
    // is almost always a mis-triangulation, further than this contributes
    // almost nothing to inverse depth and drags the fit around.
    float min_sample_depth_m = 0.3f;
    float max_sample_depth_m = 15.0f;

    // Fraction of samples that must end up as inliers for the fit to be
    // trusted at all.
    float min_inlier_ratio = 0.45f;

    // Temporal smoothing of the fitted parameters, in [0, 1]: 0 keeps the
    // previous fit, 1 takes each frame's fit whole. The scene's true scale
    // cannot change quickly, so smoothing mostly removes fit noise.
    float smoothing = 0.35f;

    // A new fit whose depths disagree with the held one by more than this
    // (as a ratio, at a reference distance) replaces it outright instead of
    // being blended in. Without this, a genuine scale change - relocalising
    // into a different map, say - would be averaged towards for many seconds.
    float reset_ratio = 2.0f;
};

// Fits and holds the relative-to-metric mapping across frames.
//
// Pure: takes a depth image and a set of metric samples, returns numbers. No
// model, no network, no hardware - which is what makes the part of this
// feature that can be wrong in a quiet, dangerous way fully testable.
class DepthScaler {
public:
    explicit DepthScaler(DepthScalerConfig config = {});

    void setConfig(const DepthScalerConfig& config) { config_ = config; }
    const DepthScalerConfig& config() const { return config_; }

    // `relative` is the network's raw output at its own resolution; the
    // samples carry the resolution their pixel coordinates are in, and are
    // rescaled to match.
    DepthScale fit(const cv::Mat& relative, const common::SparseDepthFrame& samples);

    // The held (smoothed) fit, which is what callers should convert with.
    const DepthScale& current() const { return held_; }

    void reset();

private:
    DepthScalerConfig config_;
    DepthScale held_;
    bool have_held_ = false;
};

} // namespace perception
