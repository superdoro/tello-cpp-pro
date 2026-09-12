// Milestone (a): validate Tello UDP connectivity end-to-end via keyboard
// teleop, before any video/SLAM/control code is layered on top.
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "common/logging.hpp"
#include "common/types.hpp"
#include "core/event_bus.hpp"
#include "drivers/tello/tello_driver.hpp"

namespace {

std::atomic<bool> g_running{true};

void handleSigint(int) { g_running = false; }

// Puts stdin into raw, non-blocking, non-echoing mode for the lifetime of the
// object so single keypresses can be read without waiting for Enter.
class RawTerminal {
public:
    RawTerminal() {
        tcgetattr(STDIN_FILENO, &original_);
        termios raw = original_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &original_); }

private:
    termios original_{};
};

void printHelp() {
    std::cout << "Manual Tello control (hold a key to move, release to stop)\n"
                 "  w/s      : throttle up/down     a/d      : yaw ccw/cw\n"
                 "  arrow up/down    : forward/back  arrow left/right : roll left/right\n"
                 "  space : zero velocity now        t : takeoff   l : land\n"
                 "  x : EMERGENCY STOP                ESC : quit (lands first)\n";
}

// A terminal gives no key-release event, only a stream of repeated keydowns
// while a key is held (via the OS's keyboard auto-repeat) and then silence
// once released. Each axis is therefore zeroed if its key hasn't repeated
// within kHoldTimeout. This must be longer than typical OS auto-repeat
// intervals (a few tens of ms) so a genuine hold isn't misread as released,
// but short enough that release-to-stop still feels immediate.
struct AxisActivity {
    std::chrono::steady_clock::time_point pitch{};
    std::chrono::steady_clock::time_point roll{};
    std::chrono::steady_clock::time_point throttle{};
    std::chrono::steady_clock::time_point yaw{};
};

enum class Key { None, Up, Down, Left, Right, Escape, Char };

struct KeyEvent {
    Key key = Key::None;
    char ch = 0;
};

// Reads at most one key event without blocking. Arrow keys arrive as the
// 3-byte escape sequence ESC '[' <A|B|C|D>; a lone ESC byte (nothing follows
// within a few milliseconds) is reported as Key::Escape. Without this, a lone
// `case 27: quit` would misfire on every arrow-key press, since arrow keys
// also start with byte 27.
KeyEvent readKey() {
    char c = 0;
    if (::read(STDIN_FILENO, &c, 1) != 1) return {};
    if (c != 27) return {Key::Char, c};

    char seq[2] = {0, 0};
    for (char& b : seq) {
        int attempts = 0;
        while (::read(STDIN_FILENO, &b, 1) != 1) {
            if (++attempts > 5) return {Key::Escape, 27};
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    if (seq[0] != '[') return {Key::Escape, 27};
    switch (seq[1]) {
        case 'A': return {Key::Up, 0};
        case 'B': return {Key::Down, 0};
        case 'C': return {Key::Right, 0};
        case 'D': return {Key::Left, 0};
        default: return {Key::Escape, 27};
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string ip = "192.168.10.1";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--ip") ip = argv[i + 1];
    }

    std::signal(SIGINT, handleSigint);

    core::EventBus bus;
    bus.subscribe<common::DroneState>([](const common::DroneState& state) {
        static int counter = 0;
        if (++counter % 20 == 0) {
            std::cout << "\r[state] battery=" << state.battery_pct << "% height=" << state.height_cm
                       << "cm yaw=" << state.yaw_deg << "deg      " << std::flush;
        }
    });

    drivers::tello::TelloConnectionConfig config;
    config.ip = ip;
    drivers::tello::TelloDriver drone(config, bus);

    if (!drone.connect()) {
        common::logError("manual_control", "failed to connect to Tello at " + ip);
        return 1;
    }

    printHelp();
    RawTerminal rawTerminal;

    common::VelocityCommand velocity;
    AxisActivity axisActivity;
    constexpr int kStep = 50;
    constexpr auto kTickInterval = std::chrono::milliseconds(50);
    // Must exceed the OS's initial key-repeat delay (commonly 300-600ms)
    // before a held key starts auto-repeating, or a genuine hold gets read
    // as "released" during that gap and briefly stops before repeats kick
    // in. The unavoidable trade-off: a single tap now also holds its
    // velocity for up to this long, since a tap and the start of a hold are
    // indistinguishable until a repeat either arrives or doesn't. Lower this
    // (and your terminal's own repeat delay, e.g. `xset r rate 200 30`) for
    // snappier stop-on-release at the cost of stutter on long holds.
    constexpr auto kHoldTimeout = std::chrono::milliseconds(600);

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();

        // Drain every key event queued since the last tick, not just one.
        // The OS's key auto-repeat can emit events faster than our tick
        // interval, and handling only one per tick would let a backlog build
        // up - which would then desync "current key state" from what's
        // actually held, delaying the stop-on-release this loop exists for.
        for (KeyEvent event = readKey(); event.key != Key::None; event = readKey()) {
            switch (event.key) {
                case Key::Up: velocity.pitch = kStep; axisActivity.pitch = now; break;
                case Key::Down: velocity.pitch = -kStep; axisActivity.pitch = now; break;
                case Key::Left: velocity.roll = -kStep; axisActivity.roll = now; break;
                case Key::Right: velocity.roll = kStep; axisActivity.roll = now; break;
                case Key::Escape: g_running = false; break;
                case Key::Char:
                    switch (event.ch) {
                        case 'w': velocity.throttle = kStep; axisActivity.throttle = now; break;
                        case 's': velocity.throttle = -kStep; axisActivity.throttle = now; break;
                        case 'a': velocity.yaw = -kStep; axisActivity.yaw = now; break;
                        case 'd': velocity.yaw = kStep; axisActivity.yaw = now; break;
                        case ' ': velocity = {}; break;
                        case 't': drone.takeoff(); break;
                        case 'l': drone.land(); break;
                        case 'x': drone.emergencyStop(); g_running = false; break;
                        default: break;
                    }
                    break;
                case Key::None: break;
            }
        }

        if (now - axisActivity.pitch > kHoldTimeout) velocity.pitch = 0;
        if (now - axisActivity.roll > kHoldTimeout) velocity.roll = 0;
        if (now - axisActivity.throttle > kHoldTimeout) velocity.throttle = 0;
        if (now - axisActivity.yaw > kHoldTimeout) velocity.yaw = 0;

        drone.sendVelocity(velocity);
        std::this_thread::sleep_for(kTickInterval);
    }

    drone.land();
    drone.disconnect();
    std::cout << "\nExiting.\n";
    return 0;
}
