# Tello EDU + ORB-SLAM3 Autonomous Flight

Pure C++ (no ROS), single-process/multi-threaded control stack for the Ryze
Tello EDU: connect over its SDK 2.0 UDP protocol, localize with ORB-SLAM3
against a pre-built map, and fly a pre-defined waypoint route closed-loop on
SLAM pose feedback.

Full architecture, module boundaries, coordinate-frame handling, and the
phased roadmap are in the design plan:
`~/.claude/plans/c-tello-edu-orb-slam3-slam-nifty-kahan.md`.

## Status

- [x] **(a) Tello connectivity + manual control** — `drivers/tello/`, `apps/manual_control`
- [ ] (b) Video stream decode + display
- [ ] (c) ORB-SLAM3 mapping mode
- [ ] (d) Save map + reload in localization mode
- [ ] (e) Waypoint mission + PID closed-loop control
- [ ] (f) End-to-end autonomous route flight

## Layout

```
common/         shared types, logging, queue primitives — no dependency on anything else
core/           EventBus (typed pub/sub) + module lifecycle base classes
drivers/tello/  Tello SDK 2.0 UDP implementation, behind the IDrone interface
video/          (milestone b) H264 decode via libavcodec
slam/           (milestone c/d) ORB-SLAM3 wrapper, behind the ILocalizer interface
control/        (milestone e) waypoint/mission types, PID flight controller
apps/           composition-root executables, one per milestone
third_party/    ORB-SLAM3 as a git submodule (added when BUILD_SLAM is enabled)
```

Every module talks to the others only through `core::EventBus` topics carrying
plain types from `common/types.hpp` — no module holds a pointer into another
module's internals. See the plan doc for the full rationale (it's what keeps a
future multi-process split a localized change).

## Building milestone (a)

No external dependencies beyond a C++20 compiler, CMake, and POSIX sockets.

```sh
cmake -S . -B build
cmake --build build -j
```

Connect to the Tello's own WiFi network first, then:

```sh
./build/apps/manual_control/manual_control --ip 192.168.10.1
```

Keys: `w/s` throttle up/down, `a/d` yaw ccw/cw, arrow up/down forward/back,
arrow left/right roll left/right, `space` zero velocity, `t` takeoff, `l` land,
`x` emergency stop, `Esc` quit (lands first).

## Known open risks (see plan doc for detail)

- Whether high-rate `rc` streaming alone resets Tello's ~15s auto-land timer,
  or the keep-alive ping is always needed — to verify on real hardware.
- Which ORB-SLAM3 fork/commit to pin (upstream has known build breakage on
  newer Ubuntu/OpenCV/Eigen).
- Monocular scale recovery and SLAM-frame/body-frame alignment (milestone e)
  is the highest-uncertainty part of the whole project.
