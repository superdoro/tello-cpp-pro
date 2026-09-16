#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "common/frame_alignment.hpp"
#include "common/types.hpp"

namespace common {

// What the perception stack is currently able to tell you. Anything other
// than Ok means the avoidance layer must degrade - and degrading may only
// ever slow the drone down, never steer it and never speed it up.
enum class PerceptionHealth : std::uint8_t {
    Disabled,      // not built, not configured, or switched off
    Starting,      // running, but no usable output yet (or just reset)
    Ok,
    NoDepthScale,  // models run, but the depth could not be fitted to metres
    EngineFailed,  // inference returned an error
    Overrun,       // falling so far behind that its output is not actionable
    Stopped,
};

// Per-bin state. These are flags rather than an enum because a bin is
// routinely several of them at once.
namespace bin_flags {
// There is evidence about this direction - either something was seen in it, or
// it was carved as free space. WITHOUT THIS BIT, "clear" means "invisible".
//
// This camera sees 55 degrees horizontally, so most of the 360-degree snapshot
// is unobserved at any instant and is only filled in from memory. Treating an
// unobserved bin as clear is the single most direct route from this feature to
// flying confidently into a wall.
inline constexpr std::uint8_t kObserved = 1u << 0;
inline constexpr std::uint8_t kOccupied = 1u << 1;
// Backed by a detection of a class marked dynamic (a person, typically).
// Such a bin still blocks, but must never be chosen as somewhere to fly.
inline constexpr std::uint8_t kDynamic = 1u << 2;
inline constexpr std::uint8_t kDetectionBacked = 1u << 3;
}  // namespace bin_flags

struct ObstacleBin {
    // Distance to the nearest occupied voxel in this direction, in metres.
    // Infinite means "nothing known to be there" - which is only good news
    // when kObserved is also set.
    float range_m = std::numeric_limits<float>::infinity();

    // [0, 1]. Occupancy strength folded together with how well-scaled the
    // depth behind it was.
    float confidence = 0.0f;

    std::uint8_t class_id = 255;  // 255 = none/unknown
    std::uint8_t flags = 0;

    bool observed() const { return (flags & bin_flags::kObserved) != 0; }
    bool occupied() const { return (flags & bin_flags::kOccupied) != 0; }
    bool dynamic() const { return (flags & bin_flags::kDynamic) != 0; }
};

// A polar clearance picture around the drone in the BODY frame
// (x forward, y left, z up), as of `reference_pose`.
//
// Fixed-size and trivially copyable (~1 KB) so the control loop can copy the
// whole thing under one lock rather than reading through anything perception
// owns while perception is writing it.
struct ObstacleSnapshot {
    static constexpr int kAzimuthBins = 24;    // 15 degrees each, full 360
    static constexpr int kElevationBins = 5;   // 18 degrees each, [-45, +45]
    static constexpr int kBinCount = kAzimuthBins * kElevationBins;

    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr float kAzimuthBinRad = 2.0f * kPi / kAzimuthBins;
    static constexpr float kElevationSpanRad = kPi * 0.5f;  // 90 degrees total
    static constexpr float kElevationBinRad = kElevationSpanRad / kElevationBins;

    std::array<ObstacleBin, kBinCount> bins{};

    // When the SOURCE FRAME was captured - not when this snapshot was
    // computed. Staleness must be measured against this, or the inference and
    // integration time (the larger part) goes unaccounted for.
    Timestamp stamp{};
    Timestamp computed_at{};

    // The MAP-frame pose the bins are relative to.
    Pose reference_pose{};

    std::uint64_t source_frame_index = 0;
    // Increments once per published snapshot. A sequence that stops advancing
    // says the perception thread is wedged even when health still reads Ok.
    std::uint32_t sequence = 0;

    PerceptionHealth health = PerceptionHealth::Disabled;
    float depth_scale_confidence = 0.0f;
    float latency_ms = 0.0f;
    int occupied_voxels = 0;

    // Nearest occupied range inside the forward corridor, precomputed by
    // perception so the brake decision is a single float comparison on the
    // control thread.
    float corridor_clearance_m = std::numeric_limits<float>::infinity();

    // --- bin addressing ---------------------------------------------------
    //
    // Azimuth bin i is CENTRED on i * 15 degrees, so bin 0 is straight ahead
    // and the forward corridor is symmetric about it. Positive azimuth is to
    // the left, matching the body frame's +y and the project's
    // counter-clockwise-positive heading convention.

    static int index(int azimuthBin, int elevationBin) {
        return elevationBin * kAzimuthBins + azimuthBin;
    }

    static int azimuthBin(float radians) {
        const float wrapped = radians - 2.0f * kPi * std::floor(radians / (2.0f * kPi));
        int bin = static_cast<int>(std::floor(wrapped / kAzimuthBinRad + 0.5f));
        bin %= kAzimuthBins;
        return bin < 0 ? bin + kAzimuthBins : bin;
    }

    // Elevation is clamped, not wrapped: anything steeper than +/-45 degrees
    // is not somewhere this drone flies, and folding it round would put a
    // ceiling fixture in the floor bin.
    static int elevationBin(float radians) {
        const float shifted = (radians + kElevationSpanRad * 0.5f) / kElevationBinRad;
        return std::clamp(static_cast<int>(std::floor(shifted)), 0, kElevationBins - 1);
    }

    static float azimuthBinCentre(int bin) { return wrapAngle(bin * kAzimuthBinRad); }

    static float elevationBinCentre(int bin) {
        return -kElevationSpanRad * 0.5f + (static_cast<float>(bin) + 0.5f) * kElevationBinRad;
    }

    // Unit vector in the BODY frame pointing at the centre of a bin.
    static Vector3 binDirection(int azimuthBin, int elevationBin) {
        const float az = azimuthBinCentre(azimuthBin);
        const float el = elevationBinCentre(elevationBin);
        const float horizontal = std::cos(el);
        return {horizontal * std::cos(az), horizontal * std::sin(az), std::sin(el)};
    }

    const ObstacleBin& at(int azimuthBin, int elevationBin) const {
        return bins[index(azimuthBin, elevationBin)];
    }

    ObstacleBin& at(int azimuthBin, int elevationBin) {
        return bins[index(azimuthBin, elevationBin)];
    }

    const ObstacleBin& atAngles(float azimuthRad, float elevationRad) const {
        return at(azimuthBin(azimuthRad), elevationBin(elevationRad));
    }
};

} // namespace common
