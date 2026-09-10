#include "tello_state_parser.hpp"

#include <sstream>
#include <unordered_map>

namespace drivers::tello::detail {

bool parseTelloState(const std::string& line, common::DroneState& outState) {
    std::unordered_map<std::string, std::string> fields;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ';')) {
        const auto colon = token.find(':');
        if (colon == std::string::npos) continue;
        fields[token.substr(0, colon)] = token.substr(colon + 1);
    }
    if (fields.empty()) return false;

    auto asFloat = [&](const char* key) -> float {
        auto it = fields.find(key);
        if (it == fields.end()) return 0.0f;
        try {
            return std::stof(it->second);
        } catch (const std::exception&) {
            return 0.0f;
        }
    };
    auto asInt = [&](const char* key) -> int {
        auto it = fields.find(key);
        if (it == fields.end()) return 0;
        try {
            return std::stoi(it->second);
        } catch (const std::exception&) {
            return 0;
        }
    };

    outState.pitch_deg = asFloat("pitch");
    outState.roll_deg = asFloat("roll");
    outState.yaw_deg = asFloat("yaw");
    outState.vgx = asFloat("vgx");
    outState.vgy = asFloat("vgy");
    outState.vgz = asFloat("vgz");
    outState.temp_low_c = asInt("templ");
    outState.temp_high_c = asInt("temph");
    outState.tof_cm = asFloat("tof");
    outState.height_cm = asFloat("h");
    outState.battery_pct = asInt("bat");
    outState.baro = asFloat("baro");
    outState.motor_time_s = static_cast<std::uint32_t>(asInt("time"));
    outState.agx = asFloat("agx");
    outState.agy = asFloat("agy");
    outState.agz = asFloat("agz");
    return true;
}

} // namespace drivers::tello::detail
