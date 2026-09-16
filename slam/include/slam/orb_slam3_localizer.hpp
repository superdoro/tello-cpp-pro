#pragma once

#include <memory>
#include <vector>

#include "slam/i_localizer.hpp"

namespace ORB_SLAM3 {
class MapPoint;
}

namespace Sophus {
template <typename Scalar, int Options>
class SE3;
using SE3f = SE3<float, 0>;
}  // namespace Sophus

namespace slam {

// ILocalizer backed by ORB-SLAM3 in monocular mode.
//
// Everything ORB-SLAM3 is hidden behind a pimpl: this header pulls in no
// ORB-SLAM3, Pangolin, Eigen or Sophus type, which is what lets the rest of
// the project compile as C++20 while the vendored library builds as C++14.
class OrbSlam3Localizer : public ILocalizer {
public:
    OrbSlam3Localizer();
    ~OrbSlam3Localizer() override;

    OrbSlam3Localizer(const OrbSlam3Localizer&) = delete;
    OrbSlam3Localizer& operator=(const OrbSlam3Localizer&) = delete;

    bool initialize(const LocalizerConfig& config) override;
    common::PoseEstimate processFrame(const video::Frame& frame) override;
    common::TrackingState state() const override;
    LocalizerStats stats() const override;
    bool trackedDepthSamples(common::SparseDepthFrame& out) const override;
    bool saveMap() override;
    bool saveTrajectory(const std::string& path) override;
    void shutdown() override;

private:
    // Declared here rather than in the anonymous namespace because it needs
    // Impl; the ORB_SLAM3::MapPoint parameter is only ever an incomplete
    // forward declaration to anyone including this header.
    void updateMapIdentity(const std::vector<ORB_SLAM3::MapPoint*>& trackedPoints);

    // Captured on the SLAM thread, for the reason given at the definition.
    void captureDepthSamples(const video::Frame& frame, const Sophus::SE3f& Tcw,
                              const std::vector<ORB_SLAM3::MapPoint*>& trackedPoints);
    void maybeFreezeMap();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace slam
