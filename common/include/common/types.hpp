#pragma once

#include <chrono>
#include <cstdint>

namespace common {

using Timestamp = std::chrono::steady_clock::time_point;

struct Vector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Pose {
    Vector3 position;
    Quaternion orientation;
    Timestamp stamp{};
};

// Mirrors the Tello `rc a b c d` axes; each channel is clamped to [-100, 100]
// by the driver before being sent.
struct VelocityCommand {
    int roll = 0;      // left(-) / right(+)
    int pitch = 0;     // back(-) / forward(+)
    int throttle = 0;  // down(-) / up(+)
    int yaw = 0;       // ccw(-) / cw(+)
};

// Parsed contents of a Tello state UDP packet (port 8890).
struct DroneState {
    int battery_pct = 0;
    float height_cm = 0.0f;
    float tof_cm = 0.0f;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
    float vgx = 0.0f;
    float vgy = 0.0f;
    float vgz = 0.0f;
    float agx = 0.0f;
    float agy = 0.0f;
    float agz = 0.0f;
    float baro = 0.0f;
    int temp_low_c = 0;
    int temp_high_c = 0;
    std::uint32_t motor_time_s = 0;
    Timestamp received_at{};
};

enum class TrackingState { NotInitialized, Ok, Lost, Recovering };

struct PoseEstimate {
    Pose pose;
    TrackingState state = TrackingState::NotInitialized;
    float confidence = 0.0f;
};

} // namespace common
