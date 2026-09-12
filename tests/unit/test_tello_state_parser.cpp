#include "../../drivers/tello/src/tello_state_parser.hpp"
#include "test_check.hpp"

namespace {

void testParsesRealPacket() {
    test::beginCase("a real state packet parses field by field");
    const std::string line =
        "pitch:-1;roll:2;yaw:-93;vgx:0;vgy:0;vgz:0;templ:62;temph:65;tof:10;h:0;bat:87;"
        "baro:47.65;time:0;agx:-5.00;agy:12.00;agz:-999.00;";

    common::DroneState state;
    CHECK(drivers::tello::detail::parseTelloState(line, state));
    CHECK_NEAR(state.pitch_deg, -1.0, 1e-5);
    CHECK_NEAR(state.roll_deg, 2.0, 1e-5);
    CHECK_NEAR(state.yaw_deg, -93.0, 1e-5);
    CHECK_EQ(state.battery_pct, 87);
    CHECK_NEAR(state.tof_cm, 10.0, 1e-5);
    CHECK_NEAR(state.baro, 47.65, 1e-3);
    CHECK_EQ(state.temp_low_c, 62);
    CHECK_EQ(state.temp_high_c, 65);
    CHECK_NEAR(state.agz, -999.0, 1e-3);
}

void testRejectsGarbage() {
    test::beginCase("a non-state datagram is rejected, not half-parsed");
    common::DroneState state;
    CHECK(!drivers::tello::detail::parseTelloState("ok", state));
    CHECK(!drivers::tello::detail::parseTelloState("", state));
}

void testToleratesMissingFields() {
    test::beginCase("a truncated packet still yields the fields it does carry");
    common::DroneState state;
    CHECK(drivers::tello::detail::parseTelloState("bat:42;h:15;", state));
    CHECK_EQ(state.battery_pct, 42);
    CHECK_NEAR(state.height_cm, 15.0, 1e-5);
}

}  // namespace

int main() {
    std::cout << "tello_state_parser\n";
    testParsesRealPacket();
    testRejectsGarbage();
    testToleratesMissingFields();
    return test::summary("tello_state_parser");
}
