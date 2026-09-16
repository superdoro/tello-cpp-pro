#include "perception/depth_scaler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "common/logging.hpp"

namespace perception {
namespace {

// One usable pairing of a network reading with a metric truth.
struct Pairing {
    float relative = 0.0f;     // network output at that pixel
    float inverse_metric = 0.0f;  // 1 / depth_m, the quantity that is affine
    float depth_m = 0.0f;
    float weight = 1.0f;
};

float sampleRelative(const cv::Mat& relative, float u, float v, float scaleX, float scaleY) {
    const int x = static_cast<int>(std::lround(u * scaleX));
    const int y = static_cast<int>(std::lround(v * scaleY));
    if (x < 0 || y < 0 || x >= relative.cols || y >= relative.rows) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return relative.at<float>(y, x);
}

float medianOf(std::vector<float>& values) {
    if (values.empty()) return 0.0f;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
}

}  // namespace

DepthScaler::DepthScaler(DepthScalerConfig config) : config_(std::move(config)) {}

void DepthScaler::reset() {
    held_ = {};
    have_held_ = false;
}

DepthScale DepthScaler::fit(const cv::Mat& relative, const common::SparseDepthFrame& samples) {
    DepthScale failed;

    if (relative.empty() || relative.type() != CV_32F || samples.samples.empty()) return failed;
    if (samples.image_width <= 0 || samples.image_height <= 0) return failed;

    const float scaleX = static_cast<float>(relative.cols) / static_cast<float>(samples.image_width);
    const float scaleY =
        static_cast<float>(relative.rows) / static_cast<float>(samples.image_height);

    std::vector<Pairing> pairings;
    pairings.reserve(samples.samples.size());
    for (const common::DepthSample& sample : samples.samples) {
        if (sample.depth_m < config_.min_sample_depth_m ||
            sample.depth_m > config_.max_sample_depth_m) {
            continue;
        }
        const float value = sampleRelative(relative, sample.u, sample.v, scaleX, scaleY);
        if (!std::isfinite(value)) continue;

        Pairing pairing;
        pairing.relative = value;
        pairing.depth_m = sample.depth_m;
        pairing.inverse_metric = 1.0f / sample.depth_m;
        pairing.weight = std::clamp(sample.weight, 0.0f, 1.0f);
        pairings.push_back(pairing);
    }

    failed.samples_used = static_cast<int>(pairings.size());
    if (static_cast<int>(pairings.size()) < config_.min_samples) return failed;

    // --- RANSAC over pairs -------------------------------------------------
    //
    // Two points define a line in (relative, 1/depth). Sampling pairs rather
    // than least-squaring everything matters because ORB-SLAM3's map points
    // include genuine gross outliers - a point matched onto the wrong surface
    // sits metres from the truth, and one of those is enough to drag a plain
    // least-squares fit into nonsense.
    std::mt19937 rng(12345);  // fixed: a flight should not depend on luck
    std::uniform_int_distribution<std::size_t> pick(0, pairings.size() - 1);

    float bestA = 0.0f;
    float bestB = 0.0f;
    float bestScore = -1.0f;

    const auto inlierTolerance = [this](float depth) {
        return std::max(config_.inlier_tolerance_m, config_.inlier_tolerance_fraction * depth);
    };

    for (int iteration = 0; iteration < config_.ransac_iterations; ++iteration) {
        const Pairing& p = pairings[pick(rng)];
        const Pairing& q = pairings[pick(rng)];
        const float span = q.relative - p.relative;
        // Two readings the network calls the same distance say nothing about
        // the slope.
        if (std::abs(span) < 1e-4f) continue;

        const float a = (q.inverse_metric - p.inverse_metric) / span;
        const float b = p.inverse_metric - a * p.relative;
        // Inverse depth must increase as the network reports "nearer"; a
        // negative slope is a fit to noise.
        if (!(a > 0.0f)) continue;

        float score = 0.0f;
        for (const Pairing& candidate : pairings) {
            const float inverse = a * candidate.relative + b;
            if (!(inverse > 1e-4f)) continue;
            const float metres = 1.0f / inverse;
            if (std::abs(metres - candidate.depth_m) <= inlierTolerance(candidate.depth_m)) {
                score += candidate.weight;
            }
        }
        if (score > bestScore) {
            bestScore = score;
            bestA = a;
            bestB = b;
        }
    }

    if (bestScore <= 0.0f) return failed;

    // --- refine on the inliers --------------------------------------------
    double sumW = 0.0, sumX = 0.0, sumY = 0.0, sumXX = 0.0, sumXY = 0.0;
    int inliers = 0;
    std::vector<float> errors;
    errors.reserve(pairings.size());

    for (const Pairing& candidate : pairings) {
        const float inverse = bestA * candidate.relative + bestB;
        if (!(inverse > 1e-4f)) continue;
        const float metres = 1.0f / inverse;
        if (std::abs(metres - candidate.depth_m) > inlierTolerance(candidate.depth_m)) continue;

        const double w = candidate.weight;
        sumW += w;
        sumX += w * candidate.relative;
        sumY += w * candidate.inverse_metric;
        sumXX += w * candidate.relative * candidate.relative;
        sumXY += w * candidate.relative * candidate.inverse_metric;
        ++inliers;
    }

    const double denominator = sumW * sumXX - sumX * sumX;
    float a = bestA;
    float b = bestB;
    if (std::abs(denominator) > 1e-12 && inliers >= 2) {
        const double refinedA = (sumW * sumXY - sumX * sumY) / denominator;
        const double refinedB = (sumY - refinedA * sumX) / sumW;
        if (refinedA > 0.0) {
            a = static_cast<float>(refinedA);
            b = static_cast<float>(refinedB);
        }
    }

    for (const Pairing& candidate : pairings) {
        const float inverse = a * candidate.relative + b;
        if (!(inverse > 1e-4f)) continue;
        errors.push_back(std::abs(1.0f / inverse - candidate.depth_m));
    }

    DepthScale fitted;
    fitted.a = a;
    fitted.b = b;
    fitted.inliers = inliers;
    fitted.samples_used = static_cast<int>(pairings.size());
    fitted.median_error_m = medianOf(errors);

    const float inlierRatio =
        static_cast<float>(inliers) / static_cast<float>(pairings.size());
    if (inlierRatio < config_.min_inlier_ratio) {
        // Report the numbers, but with zero confidence: the avoidance layer
        // treats that as "slow down, do not steer", which is the right
        // response to a scale nobody can vouch for.
        fitted.confidence = 0.0f;
        return fitted;
    }

    // Confidence blends how much of the evidence agreed with how much
    // evidence there was. A tight fit to nine points is not as trustworthy as
    // a tight fit to ninety.
    const float countFactor =
        std::clamp(static_cast<float>(inliers) / 40.0f, 0.0f, 1.0f);
    fitted.confidence = std::clamp(inlierRatio * (0.4f + 0.6f * countFactor), 0.0f, 1.0f);

    // --- hold and smooth ---------------------------------------------------
    if (!have_held_) {
        held_ = fitted;
        have_held_ = true;
        return held_;
    }

    // A fit that disagrees wildly with the held one is a real change, not
    // noise: adopt it rather than crawling towards it over several seconds.
    constexpr float kReferenceRelative = 0.5f;
    const float heldMetres = held_.metres(kReferenceRelative);
    const float newMetres = fitted.metres(kReferenceRelative);
    const bool bothFinite = std::isfinite(heldMetres) && std::isfinite(newMetres);
    const float ratio = bothFinite && heldMetres > 0.0f && newMetres > 0.0f
                             ? std::max(heldMetres / newMetres, newMetres / heldMetres)
                             : std::numeric_limits<float>::infinity();

    if (ratio > config_.reset_ratio) {
        common::logWarn("DepthScaler",
                         "depth scale changed abruptly - adopting the new fit rather than "
                         "blending towards it");
        held_ = fitted;
        return held_;
    }

    const float alpha = std::clamp(config_.smoothing, 0.0f, 1.0f);
    held_.a += alpha * (fitted.a - held_.a);
    held_.b += alpha * (fitted.b - held_.b);
    held_.confidence = fitted.confidence;
    held_.inliers = fitted.inliers;
    held_.samples_used = fitted.samples_used;
    held_.median_error_m = fitted.median_error_m;
    return held_;
}

} // namespace perception
