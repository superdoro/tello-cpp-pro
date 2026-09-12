// Milestone (a) alternative front-end: an SDL2 window gives real key
// down/up events, unlike a terminal which only reports repeated keydowns
// while a key is held and nothing on release. That removes the whole
// "how long to wait before assuming the key was released" trade-off that
// apps/manual_control has to guess at. Reuses the same IDrone/TelloDriver as
// the terminal version - only the input front-end differs.
#include <SDL2/SDL.h>

#include <iostream>
#include <string>

#include "common/logging.hpp"
#include "common/types.hpp"
#include "core/event_bus.hpp"
#include "drivers/tello/tello_driver.hpp"

namespace {

constexpr int kStep = 50;
constexpr Uint32 kSendIntervalMs = 50;
constexpr int kWindowWidth = 480;
constexpr int kWindowHeight = 360;

// One boolean per physical key, rather than per axis, so opposite keys
// (e.g. W/S) combine correctly regardless of press/release order: releasing
// one while the other is still held leaves the axis driven by the one still
// down instead of snapping to zero.
struct KeyState {
    bool up = false, down = false, left = false, right = false;
    bool w = false, s = false, a = false, d = false;
};

common::VelocityCommand computeVelocity(const KeyState& k) {
    common::VelocityCommand v;
    v.pitch = (static_cast<int>(k.up) - static_cast<int>(k.down)) * kStep;
    v.roll = (static_cast<int>(k.right) - static_cast<int>(k.left)) * kStep;
    v.throttle = (static_cast<int>(k.w) - static_cast<int>(k.s)) * kStep;
    v.yaw = (static_cast<int>(k.d) - static_cast<int>(k.a)) * kStep;
    return v;
}

void drawAxisBar(SDL_Renderer* renderer, int x, int y, int w, int h, int value) {
    SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
    SDL_Rect track{x, y, w, h};
    SDL_RenderFillRect(renderer, &track);

    SDL_SetRenderDrawColor(renderer, 80, 200, 120, 255);
    const int mid = x + w / 2;
    const int extent = (value * (w / 2)) / 100;
    SDL_Rect fill = (extent >= 0) ? SDL_Rect{mid, y, extent, h} : SDL_Rect{mid + extent, y, -extent, h};
    SDL_RenderFillRect(renderer, &fill);

    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    SDL_RenderDrawLine(renderer, mid, y, mid, y + h);
}

} // namespace

int main(int argc, char** argv) {
    std::string ip = "192.168.10.1";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--ip") ip = argv[i + 1];
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        common::logError("manual_control_gui", std::string("SDL_Init failed: ") + SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Tello Manual Control (SDL)", SDL_WINDOWPOS_CENTERED,
                                           SDL_WINDOWPOS_CENTERED, kWindowWidth, kWindowHeight,
                                           SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!window || !renderer) {
        common::logError("manual_control_gui", std::string("SDL window/renderer creation failed: ") + SDL_GetError());
        SDL_Quit();
        return 1;
    }

    std::cout << "Tello Manual Control (SDL window must have focus to receive keys)\n"
                 "  w/s : throttle up/down     a/d : yaw ccw/cw\n"
                 "  arrow keys : forward/back/roll left/right\n"
                 "  space : zero velocity now  t : takeoff   l : land\n"
                 "  x : EMERGENCY STOP          Esc/close window : quit (lands first)\n"
                 "Bars top to bottom: pitch, roll, throttle, yaw, battery.\n";

    core::EventBus bus;
    common::DroneState latestState;
    bus.subscribe<common::DroneState>([&](const common::DroneState& state) { latestState = state; });

    drivers::tello::TelloConnectionConfig config;
    config.ip = ip;
    drivers::tello::TelloDriver drone(config, bus);

    if (!drone.connect()) {
        common::logError("manual_control_gui", "failed to connect to Tello at " + ip);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    KeyState keys;
    bool running = true;
    Uint32 lastSend = 0;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
                const bool pressed = (event.type == SDL_KEYDOWN);
                switch (event.key.keysym.sym) {
                    case SDLK_UP: keys.up = pressed; break;
                    case SDLK_DOWN: keys.down = pressed; break;
                    case SDLK_LEFT: keys.left = pressed; break;
                    case SDLK_RIGHT: keys.right = pressed; break;
                    case SDLK_w: keys.w = pressed; break;
                    case SDLK_s: keys.s = pressed; break;
                    case SDLK_a: keys.a = pressed; break;
                    case SDLK_d: keys.d = pressed; break;
                    case SDLK_SPACE:
                        if (pressed) keys = {};
                        break;
                    case SDLK_t:
                        if (pressed) drone.takeoff();
                        break;
                    case SDLK_l:
                        if (pressed) drone.land();
                        break;
                    case SDLK_x:
                        if (pressed) {
                            drone.emergencyStop();
                            running = false;
                        }
                        break;
                    case SDLK_ESCAPE:
                        if (pressed) running = false;
                        break;
                    default: break;
                }
            }
        }

        const Uint32 now = SDL_GetTicks();
        if (now - lastSend >= kSendIntervalMs) {
            drone.sendVelocity(computeVelocity(keys));
            lastSend = now;
        }

        SDL_SetRenderDrawColor(renderer, 24, 24, 28, 255);
        SDL_RenderClear(renderer);

        const common::VelocityCommand v = computeVelocity(keys);
        drawAxisBar(renderer, 40, 40, 400, 24, v.pitch);
        drawAxisBar(renderer, 40, 84, 400, 24, v.roll);
        drawAxisBar(renderer, 40, 128, 400, 24, v.throttle);
        drawAxisBar(renderer, 40, 172, 400, 24, v.yaw);

        // Battery bar doubles as a coarse status readout: red once low.
        const bool lowBattery = latestState.battery_pct > 0 && latestState.battery_pct < 20;
        SDL_SetRenderDrawColor(renderer, lowBattery ? 220 : 60, lowBattery ? 60 : 60, 60, 255);
        SDL_Rect batteryTrack{40, 230, 400, 24};
        SDL_RenderFillRect(renderer, &batteryTrack);
        SDL_SetRenderDrawColor(renderer, lowBattery ? 220 : 80, lowBattery ? 60 : 200, lowBattery ? 60 : 120, 255);
        SDL_Rect batteryFill{40, 230, (latestState.battery_pct * 400) / 100, 24};
        SDL_RenderFillRect(renderer, &batteryFill);

        SDL_RenderPresent(renderer);
        SDL_Delay(10);
    }

    common::logInfo("manual_control_gui",
                     "battery=" + std::to_string(latestState.battery_pct) +
                         "% height=" + std::to_string(latestState.height_cm) + "cm");

    drone.land();
    drone.disconnect();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
