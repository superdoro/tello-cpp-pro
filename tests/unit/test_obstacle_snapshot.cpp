#include <cmath>
#include <limits>

#include "common/obstacle.hpp"
#include "test_check.hpp"

namespace {

using common::ObstacleSnapshot;

constexpr float kPi = ObstacleSnapshot::kPi;

float deg(float degrees) { return degrees * kPi / 180.0f; }

void testDefaultBinIsUnknownNotClear() {
    test::beginCase("a default bin reads as unknown, not as clear");
    // The distinction the whole steer layer rests on: an untouched bin has
    // infinite range but is NOT observed, so it must never be flown into.
    const ObstacleSnapshot snapshot;
    for (const auto& bin : snapshot.bins) {
        CHECK(std::isinf(bin.range_m));
        CHECK(!bin.observed());
        CHECK(!bin.occupied());
        CHECK(!bin.dynamic());
    }
    CHECK(std::isinf(snapshot.corridor_clearance_m));
    CHECK(snapshot.health == common::PerceptionHealth::Disabled);
}

void testAzimuthBinZeroIsStraightAhead() {
    test::beginCase("azimuth bin 0 is centred on straight ahead");
    CHECK_EQ(ObstacleSnapshot::azimuthBin(0.0f), 0);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(7.4f)), 0);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(-7.4f)), 0);
    // Just past the half-bin boundary it moves on.
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(7.6f)), 1);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(-7.6f)), ObstacleSnapshot::kAzimuthBins - 1);
}

void testAzimuthIsCounterClockwisePositive() {
    test::beginCase("positive azimuth is to the left, matching the body frame");
    // Body frame is x forward, y left, so atan2(y, x) > 0 means left, and the
    // bin numbering has to agree or every avoidance manoeuvre goes the wrong way.
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(90.0f)), 6);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(180.0f)), 12);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(270.0f)), 18);
}

void testAzimuthWrapsRatherThanClamping() {
    test::beginCase("azimuth wraps at +/-180 instead of saturating");
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(360.0f)), 0);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(720.0f)), 0);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(-360.0f)), 0);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(-90.0f)), 18);
    CHECK_EQ(ObstacleSnapshot::azimuthBin(deg(370.0f)), ObstacleSnapshot::azimuthBin(deg(10.0f)));
}

void testAzimuthBinRoundTrips() {
    test::beginCase("every azimuth bin centre maps back to its own bin");
    for (int i = 0; i < ObstacleSnapshot::kAzimuthBins; ++i) {
        CHECK_EQ(ObstacleSnapshot::azimuthBin(ObstacleSnapshot::azimuthBinCentre(i)), i);
    }
}

void testElevationBinCentreIsLevel() {
    test::beginCase("the middle elevation bin is level flight");
    CHECK_EQ(ObstacleSnapshot::elevationBin(0.0f), 2);
    CHECK_NEAR(ObstacleSnapshot::elevationBinCentre(2), 0.0, 1e-6);
    CHECK_NEAR(ObstacleSnapshot::elevationBinCentre(0), deg(-36.0f), 1e-5);
    CHECK_NEAR(ObstacleSnapshot::elevationBinCentre(4), deg(36.0f), 1e-5);
}

void testElevationClampsRatherThanWrapping() {
    test::beginCase("elevation clamps outside +/-45 degrees");
    // Wrapping here would put something on the ceiling into the floor bin.
    CHECK_EQ(ObstacleSnapshot::elevationBin(deg(89.0f)), 4);
    CHECK_EQ(ObstacleSnapshot::elevationBin(deg(-89.0f)), 0);
    CHECK_EQ(ObstacleSnapshot::elevationBin(deg(1000.0f)), 4);
    CHECK_EQ(ObstacleSnapshot::elevationBin(deg(-1000.0f)), 0);
}

void testElevationBinRoundTrips() {
    test::beginCase("every elevation bin centre maps back to its own bin");
    for (int j = 0; j < ObstacleSnapshot::kElevationBins; ++j) {
        CHECK_EQ(ObstacleSnapshot::elevationBin(ObstacleSnapshot::elevationBinCentre(j)), j);
    }
}

void testIndexCoversEveryBinExactlyOnce() {
    test::beginCase("index() is a bijection onto the bin array");
    bool seen[ObstacleSnapshot::kBinCount] = {};
    for (int j = 0; j < ObstacleSnapshot::kElevationBins; ++j) {
        for (int i = 0; i < ObstacleSnapshot::kAzimuthBins; ++i) {
            const int index = ObstacleSnapshot::index(i, j);
            CHECK(index >= 0 && index < ObstacleSnapshot::kBinCount);
            CHECK(!seen[index]);
            seen[index] = true;
        }
    }
    for (bool hit : seen) CHECK(hit);
}

void testBinDirectionMatchesTheBodyFrame() {
    test::beginCase("bin directions agree with body axes (x fwd, y left, z up)");
    const common::Vector3 ahead = ObstacleSnapshot::binDirection(0, 2);
    CHECK_NEAR(ahead.x, 1.0, 1e-5);
    CHECK_NEAR(ahead.y, 0.0, 1e-5);
    CHECK_NEAR(ahead.z, 0.0, 1e-5);

    const common::Vector3 left = ObstacleSnapshot::binDirection(6, 2);
    CHECK_NEAR(left.x, 0.0, 1e-5);
    CHECK_NEAR(left.y, 1.0, 1e-5);
    CHECK_NEAR(left.z, 0.0, 1e-5);

    const common::Vector3 behind = ObstacleSnapshot::binDirection(12, 2);
    CHECK_NEAR(behind.x, -1.0, 1e-5);
    CHECK_NEAR(behind.y, 0.0, 1e-5);

    // Up-tilted bins lean up, down-tilted bins lean down, and all stay unit.
    CHECK(ObstacleSnapshot::binDirection(0, 4).z > 0.5f);
    CHECK(ObstacleSnapshot::binDirection(0, 0).z < -0.5f);
    for (int j = 0; j < ObstacleSnapshot::kElevationBins; ++j) {
        for (int i = 0; i < ObstacleSnapshot::kAzimuthBins; ++i) {
            const common::Vector3 d = ObstacleSnapshot::binDirection(i, j);
            CHECK_NEAR(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), 1.0, 1e-5);
        }
    }
}

void testAccessorsAddressTheSameBin() {
    test::beginCase("at() and atAngles() reach the same storage");
    ObstacleSnapshot snapshot;
    snapshot.at(3, 1).range_m = 1.25f;
    snapshot.at(3, 1).flags = common::bin_flags::kObserved | common::bin_flags::kOccupied;

    const float az = ObstacleSnapshot::azimuthBinCentre(3);
    const float el = ObstacleSnapshot::elevationBinCentre(1);
    CHECK_NEAR(snapshot.atAngles(az, el).range_m, 1.25, 1e-6);
    CHECK(snapshot.atAngles(az, el).observed());
    CHECK(snapshot.atAngles(az, el).occupied());
    CHECK(!snapshot.atAngles(az, el).dynamic());

    // Nothing else moved.
    CHECK(std::isinf(snapshot.at(4, 1).range_m));
}

}  // namespace

int main() {
    std::cout << "obstacle snapshot\n";
    testDefaultBinIsUnknownNotClear();
    testAzimuthBinZeroIsStraightAhead();
    testAzimuthIsCounterClockwisePositive();
    testAzimuthWrapsRatherThanClamping();
    testAzimuthBinRoundTrips();
    testElevationBinCentreIsLevel();
    testElevationClampsRatherThanWrapping();
    testElevationBinRoundTrips();
    testIndexCoversEveryBinExactlyOnce();
    testBinDirectionMatchesTheBodyFrame();
    testAccessorsAddressTheSameBin();
    return test::summary("obstacle snapshot");
}
