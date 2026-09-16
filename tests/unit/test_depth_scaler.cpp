// Fitting a monocular network's output to metres.
//
// This is the quietest dangerous code in the obstacle-avoidance stack: a
// wrong scale does not throw, log, or look wrong on screen. It just reports
// that the wall is four metres away when it is one.
#include <cmath>
#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "perception/depth_scaler.hpp"
#include "test_check.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

// Builds a network-style output for a scene whose true depths are known.
//
// The network emits INVERSE depth up to an unknown scale and shift, so the
// ground truth is inverted here rather than used directly - which is also
// what makes the "affine in inverse depth, not in depth" property testable.
cv::Mat relativeFromDepths(const std::vector<cv::Point2f>& pixels,
                            const std::vector<float>& depths, float a, float b) {
    cv::Mat relative(kHeight, kWidth, CV_32F, cv::Scalar(0.0f));
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        const float value = (1.0f / depths[i] - b) / a;
        relative.at<float>(static_cast<int>(pixels[i].y), static_cast<int>(pixels[i].x)) = value;
    }
    return relative;
}

common::SparseDepthFrame samplesFrom(const std::vector<cv::Point2f>& pixels,
                                      const std::vector<float>& depths) {
    common::SparseDepthFrame frame;
    frame.image_width = kWidth;
    frame.image_height = kHeight;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        common::DepthSample sample;
        sample.u = pixels[i].x;
        sample.v = pixels[i].y;
        sample.depth_m = depths[i];
        sample.weight = 1.0f;
        frame.samples.push_back(sample);
    }
    return frame;
}

// A corridor: structure both near and far, which is the case that separates a
// correct inverse-depth fit from a plausible-looking wrong one.
void corridorScene(std::vector<cv::Point2f>& pixels, std::vector<float>& depths, int count = 40) {
    pixels.clear();
    depths.clear();
    for (int i = 0; i < count; ++i) {
        pixels.emplace_back(static_cast<float>(10 + (i * 7) % (kWidth - 20)),
                             static_cast<float>(10 + (i * 11) % (kHeight - 20)));
        // 0.6m out to 9m.
        depths.push_back(0.6f + 8.4f * static_cast<float>(i) / static_cast<float>(count - 1));
    }
}

void testRecoversMetresFromAPerfectScene() {
    test::beginCase("a clean scene recovers the true depths");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths);

    const float trueA = 0.42f;
    const float trueB = 0.05f;
    const cv::Mat relative = relativeFromDepths(pixels, depths, trueA, trueB);

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samplesFrom(pixels, depths));

    CHECK(fit.valid());
    CHECK_NEAR(fit.a, trueA, 0.02);
    CHECK_NEAR(fit.b, trueB, 0.02);

    // What actually matters: the metres it reports.
    for (std::size_t i = 0; i < pixels.size(); i += 7) {
        const float value =
            relative.at<float>(static_cast<int>(pixels[i].y), static_cast<int>(pixels[i].x));
        CHECK_NEAR(fit.metres(value), depths[i], 0.1);
    }
}

void testSurvivesGrossOutliers() {
    test::beginCase("mis-triangulated map points do not drag the fit off");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths, 60);

    const float trueA = 0.42f;
    const float trueB = 0.05f;
    const cv::Mat relative = relativeFromDepths(pixels, depths, trueA, trueB);

    // Corrupt a third of the METRIC samples, leaving the image untouched -
    // exactly what a map point matched onto the wrong surface looks like.
    common::SparseDepthFrame samples = samplesFrom(pixels, depths);
    for (std::size_t i = 0; i < samples.samples.size(); i += 3) {
        samples.samples[i].depth_m *= 3.5f;
    }

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samples);

    CHECK(fit.valid());
    CHECK_NEAR(fit.a, trueA, 0.05);
    // The uncorrupted samples must still come back right.
    for (std::size_t i = 1; i < pixels.size(); i += 3) {
        const float value =
            relative.at<float>(static_cast<int>(pixels[i].y), static_cast<int>(pixels[i].x));
        CHECK_NEAR(fit.metres(value), depths[i], 0.35);
    }
}

void testRefusesWhenThereIsTooLittleEvidence() {
    test::beginCase("too few samples produces no fit at all, not a guess");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths, 5);

    const cv::Mat relative = relativeFromDepths(pixels, depths, 0.42f, 0.05f);

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samplesFrom(pixels, depths));

    CHECK(!fit.valid());
    CHECK_NEAR(fit.confidence, 0.0, 1e-6);
}

void testRefusesASceneAtOneDistance() {
    test::beginCase("a scene with no depth spread cannot pin down the scale");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    for (int i = 0; i < 40; ++i) {
        pixels.emplace_back(static_cast<float>(10 + (i * 7) % (kWidth - 20)),
                             static_cast<float>(10 + (i * 11) % (kHeight - 20)));
        depths.push_back(3.0f);  // a flat wall, dead ahead, nothing else
    }
    const cv::Mat relative = relativeFromDepths(pixels, depths, 0.42f, 0.05f);

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samplesFrom(pixels, depths));

    // Every pairing is degenerate, so no slope can be estimated. Anything
    // other than "no fit" here would be an invented scale.
    CHECK(!fit.valid());
}

void testIncoherentSamplesProduceNoConfidentFit() {
    test::beginCase("evidence that disagrees with itself yields no confident fit");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths, 40);
    const cv::Mat relative = relativeFromDepths(pixels, depths, 0.42f, 0.05f);

    // Each sample is wrong by a DIFFERENT factor, and all stay inside the
    // plausible depth range so the pre-filter cannot quietly rescue them.
    //
    // The distinction matters: a shared factor would not be disagreement at
    // all. Scaling every depth by the same c maps 1/d to (a*rel + b)/c, which
    // is still an exact line - RANSAC would find it and be right to. Only
    // incoherent error leaves no line to find.
    common::SparseDepthFrame samples = samplesFrom(pixels, depths);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> factor(0.35f, 2.6f);
    for (std::size_t i = 0; i < samples.samples.size(); ++i) {
        if (i % 4 == 0) continue;  // leave a minority intact
        samples.samples[i].depth_m =
            std::clamp(samples.samples[i].depth_m * factor(rng), 0.4f, 14.0f);
    }

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samples);
    CHECK(!fit.valid());
    CHECK(fit.samples_used > 0);  // it saw the evidence, it just would not vouch for it
}

void testAConsistentlyWrongScaleIsStillFitted() {
    test::beginCase("uniformly scaled samples fit cleanly - and that is not a failure");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths, 40);
    const cv::Mat relative = relativeFromDepths(pixels, depths, 0.42f, 0.05f);

    // Every sample twice its true depth: this is what an uncalibrated map
    // scale looks like, and it is a perfectly consistent line. The scaler
    // cannot and should not detect it - only tools/calibrate_scale can. The
    // test records that boundary rather than pretending otherwise.
    common::SparseDepthFrame samples = samplesFrom(pixels, depths);
    for (auto& sample : samples.samples) sample.depth_m *= 2.0f;

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samples);
    CHECK(fit.valid());
    for (std::size_t i = 0; i < pixels.size(); i += 9) {
        const float value =
            relative.at<float>(static_cast<int>(pixels[i].y), static_cast<int>(pixels[i].x));
        CHECK_NEAR(fit.metres(value), depths[i] * 2.0f, 0.3);
    }
}

void testSmoothsSmallChangesButAdoptsLargeOnes() {
    test::beginCase("the held fit smooths noise but jumps on a real scale change");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths, 40);

    perception::DepthScaler scaler;
    const perception::DepthScale first =
        scaler.fit(relativeFromDepths(pixels, depths, 0.42f, 0.05f), samplesFrom(pixels, depths));
    CHECK(first.valid());

    // A slightly different fit should move the held value only part of the way.
    const perception::DepthScale nudged =
        scaler.fit(relativeFromDepths(pixels, depths, 0.46f, 0.05f), samplesFrom(pixels, depths));
    CHECK(nudged.a > first.a);
    CHECK(nudged.a < 0.46f);  // blended, not adopted whole

    // A scene at a completely different scale must be taken at once: waiting
    // several seconds to believe it is how a drone flies into something.
    std::vector<float> muchCloser;
    for (float d : depths) muchCloser.push_back(d * 0.2f);
    const perception::DepthScale jumped = scaler.fit(
        relativeFromDepths(pixels, muchCloser, 0.42f, 0.05f), samplesFrom(pixels, muchCloser));
    CHECK(jumped.valid());
    for (std::size_t i = 0; i < pixels.size(); i += 9) {
        const float value = (1.0f / muchCloser[i] - 0.05f) / 0.42f;
        CHECK_NEAR(jumped.metres(value), muchCloser[i], 0.2);
    }
}

void testResolutionMismatchIsHandled() {
    test::beginCase("samples in full-res pixels map onto a smaller depth image");
    std::vector<cv::Point2f> pixels;
    std::vector<float> depths;
    corridorScene(pixels, depths, 40);
    const cv::Mat relative = relativeFromDepths(pixels, depths, 0.42f, 0.05f);

    // The network ran at 320x240; the samples are reported at 960x720.
    common::SparseDepthFrame samples = samplesFrom(pixels, depths);
    samples.image_width = kWidth * 3;
    samples.image_height = kHeight * 3;
    for (auto& sample : samples.samples) {
        sample.u *= 3.0f;
        sample.v *= 3.0f;
    }

    perception::DepthScaler scaler;
    const perception::DepthScale fit = scaler.fit(relative, samples);
    CHECK(fit.valid());
    CHECK_NEAR(fit.a, 0.42, 0.03);
}

}  // namespace

int main() {
    std::cout << "depth_scaler\n";
    testRecoversMetresFromAPerfectScene();
    testSurvivesGrossOutliers();
    testRefusesWhenThereIsTooLittleEvidence();
    testRefusesASceneAtOneDistance();
    testIncoherentSamplesProduceNoConfidentFit();
    testAConsistentlyWrongScaleIsStillFitted();
    testSmoothsSmallChangesButAdoptsLargeOnes();
    testResolutionMismatchIsHandled();
    return test::summary("depth_scaler");
}
