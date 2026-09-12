# Tello EDU + ORB-SLAM3 Autonomous Flight

Pure C++ (no ROS), single-process/multi-threaded control stack for the 
Tello EDU. It builds a visual map from **recorded video**, then uses that map
to localize the drone in flight and fly a pre-authored route closed-loop on
SLAM pose feedback.

Flown successfully at 2 m/s on real hardware.

---

## 1. Build

### 1.1 System packages

```sh
sudo apt install build-essential cmake git \
                 libopencv-dev libeigen3-dev \
                 libboost-serialization-dev libssl-dev \
                 libavcodec-dev libavutil-dev libswscale-dev \
                 libsdl2-dev \
                 libglew-dev libepoxy-dev libgl1-mesa-dev
```

`libsdl2-dev` is only needed for `manual_control_gui`; `libglew-dev` /
`libepoxy-dev` are for Pangolin, which ORB-SLAM3's viewer uses.

### 1.2 Submodules

```sh
git submodule update --init --recursive
```

Brings in `third_party/ORB_SLAM3` and `third_party/Pangolin`.

### 1.3 Pangolin (once)

Not installed system-wide; built in place and found there by
`third_party/CMakeLists.txt`.

```sh
cmake -S third_party/Pangolin -B third_party/Pangolin/build \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
cmake --build third_party/Pangolin/build -j
```

All of `pango_core`, `pango_display`, `pango_plot`, `pango_tools` and
`pango_video` must build - a partial Pangolin build fails later at link time.

### 1.4 ORB vocabulary (once)

```sh
./scripts/fetch_vocabulary.sh
```

Extracts `ORBvoc.txt` (~145 MB) from the tarball in the ORB-SLAM3 submodule.
The **same** file must be used for building a map and for localizing against
it: ORB-SLAM3 stores its checksum in the map and rejects a mismatch.

### 1.5 Configure and build

```sh
cmake -S . -B build -DBUILD_SLAM=ON -DBUILD_TOOLS=ON -DBUILD_TESTS=ON
cmake --build build -j
```

| CMake option | Default | What it adds |
|---|---|---|
| `BUILD_SLAM` | `OFF` | `slam/`, `map_builder`, `relocalize`, `autonomous_mission`. Needs the ORB-SLAM3 submodule and Pangolin; this is the slow part of the build. |
| `BUILD_TOOLS` | `OFF` | `calibrate_camera`, `calibrate_scale`, `trajectory_to_mission`. Requires `BUILD_SLAM`. |
| `BUILD_TESTS` | `OFF` | Unit tests plus `make_synthetic_video` and `compare_trajectory`. |
| `CMAKE_BUILD_TYPE` | `RelWithDebInfo` | Standard CMake. |

Without any options, `cmake -S . -B build && cmake --build build -j` builds
just `manual_control`, `manual_control_gui` and `video_view` - useful for
working on the drone link with no SLAM build.

Build a single target with, for example,
`cmake --build build -j --target map_builder`.

### 1.6 Firewall

The drone sends video and telemetry as **unsolicited inbound UDP**, which a
default-deny firewall drops. The command channel still works (its replies are
conntrack-ESTABLISHED), so the symptom is a drone that connects but sends no
video and reports no battery.

```sh
sudo ufw allow from 192.168.10.0/24 to any port 11111 proto udp comment 'Tello video'
sudo ufw allow from 192.168.10.0/24 to any port 8890  proto udp comment 'Tello state'
```

Scoped to the Tello's own subnet, so these ports stay closed on every other
network.

---

## 2. The workflow

```
  1. calibrate the camera      apps/tools/calibrate_camera
  2. record a route            apps/video_view --record
  3. build a map               apps/map_builder            <- from the video
  4. measure the map's scale   apps/tools/calibrate_scale
  5. verify localization       apps/relocalize
  6. author a route            apps/tools/trajectory_to_mission
  7. fly it                    apps/autonomous_mission
```

Steps 3, 5 (offline) and 6 need no drone and no battery.

**Build the map under the lighting you will fly in.** Measured on this
project's own footage, the same room at midday and in the evening differed by
30% in contrast and 16% in ORB feature count, and a map built at 11:50
relocalized 96.6% of frames from an independent 13:39 recording but **not one
frame** from 19:17. Keep a separate map per lighting condition.

---

## 3. Executables

Every path below assumes the build directory is `build/`.

### 3.1 `manual_control` - keyboard teleop, terminal

```sh
./build/apps/manual_control/manual_control [--ip 192.168.10.1]
```

| Option | Default | Meaning |
|---|---|---|
| `--ip <addr>` | `192.168.10.1` | Drone address. |

Keys: `w`/`s` throttle, `a`/`d` yaw, arrow up/down forward/back, arrow
left/right roll, `space` zero velocity, `t` takeoff, `l` land, `x` emergency
stop, `Esc` quit (lands first).

A terminal gives no key-release event, so each axis auto-zeroes ~600 ms after
its key stops repeating. Prefer the GUI variant below.

### 3.2 `manual_control_gui` - keyboard teleop, SDL2 window

```sh
./build/apps/manual_control_gui/manual_control_gui [--ip 192.168.10.1]
```

| Option | Default | Meaning |
|---|---|---|
| `--ip <addr>` | `192.168.10.1` | Drone address. |

Same keys, but a window gets real key-up events, so movement starts and stops
exactly when you press and release. The window must have keyboard focus. The
four bars are pitch/roll/throttle/yaw; the bottom bar is battery (red under
20%).

### 3.3 `video_view` - view and record the stream

```sh
./build/apps/video_view/video_view --record recordings/route.h264
```

| Option | Default | Meaning |
|---|---|---|
| `--ip <addr>` | `192.168.10.1` | Drone address. |
| `--record <file>` | none | Save the raw H264 elementary stream. This is what `map_builder` consumes. |
| `--width <n>` `--height <n>` | native | Resize frames before display and recording. |

`Esc` or Ctrl-C quits. The HUD shows resolution, measured fps, frame index,
battery, decoder errors and dropped frames; the summary on exit reports
datagrams, bytes, decoded frames and decode errors. Set `TELLO_FFMPEG_LOG=1`
to see libavcodec's own complaints, which are suppressed by default.

### 3.4 `map_builder` - build a map from video

```sh
./build/apps/map_builder/map_builder \
    --video recordings/route.h264 \
    --camera config/tello_camera.yaml \
    --out maps/office.osa --no-viewer
```

| Option | Default | Meaning |
|---|---|---|
| `--video <file>` | required | Input video: raw `.h264` or any container OpenCV can open. |
| `--out <map.osa>` | required | Where to write the map. |
| `--extend <map.osa>` | none | Add this footage to an existing map, keeping its coordinate frame and scale. Must differ from `--out`. Only useful if place recognition matches the two - it will say if it did not. |
| `--camera <yaml>` | `config/tello_camera.yaml` | Camera intrinsics. |
| `--vocabulary <file>` | `third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt` | ORB vocabulary. |
| `--trajectory <file>` | `<map>_trajectory.txt` | Where to write the keyframe trajectory (step 6 reads it). |
| `--stride <n>` | `1` | Use every nth frame. |
| `--scale <m>` | uncalibrated | Metres per SLAM unit, if already known. |
| `--no-viewer` | off | No Pangolin window. **Recommended** - see the known issue in section 7. |
| `--preview` | off | Show the input frames in an OpenCV window. |

Writes three files: the map, a `<map>.meta.yaml` sidecar (scale, provenance,
keyframe count) and the trajectory. Aim for a tracked-frame percentage in the
high 90s.

### 3.5 `relocalize` - verify a map

```sh
./build/apps/relocalize/relocalize --map maps/office.osa --video recordings/route.h264
./build/apps/relocalize/relocalize --map maps/office.osa --drone
```

| Option | Default | Meaning |
|---|---|---|
| `--map <map.osa>` | required | Map to localize against. |
| `--video <file>` | — | Localize against a recording. |
| `--drone` | — | Localize against the live stream. One of `--video`/`--drone` is required. |
| `--ip <addr>` | `192.168.10.1` | Drone address. |
| `--camera <yaml>` | `config/tello_camera.yaml` | Camera intrinsics. |
| `--vocabulary <file>` | `.../ORBvoc.txt` | ORB vocabulary. |
| `--record <file>` | none | (live) Also save the raw stream. |
| `--realtime` | off | (video) Replay at the recording's own frame rate. |
| `--freeze` | off | Switch to ORB-SLAM3's localization-only mode once settled. **Measures far worse** - see section 6. |
| `--no-viewer` | off | No Pangolin window. |
| `--no-preview` | off | No OpenCV window (headless). |

Verify in three escalating steps, and do not skip ahead: against the footage
the map was built from, then against a **different** recording of the same
space, then live with the drone on the floor.

### 3.6 `autonomous_mission` - fly the route

```sh
./build/apps/autonomous_mission/autonomous_mission \
    --map maps/office.osa --mission config/mission.yaml \
    --max-speed 100 --cruise 40 \
    --corner-limit 5 --corner-brake 4.0 --corner-min 0.2 \
    --centering 1.0 \
    --record recordings/flight.h264 --no-viewer
```

**Connection and inputs**

| Option | Default | Meaning |
|---|---|---|
| `--map <map.osa>` | required | Map to localize against. |
| `--mission <yaml>` | required | Route, from `trajectory_to_mission`. Refuses to fly a route authored against a different map. |
| `--camera <yaml>` | `config/tello_camera.yaml` | Camera intrinsics. |
| `--vocabulary <file>` | `.../ORBvoc.txt` | ORB vocabulary. |
| `--ip <addr>` | `192.168.10.1` | Drone address. |
| `--record <file>` | none | Save the live stream for post-flight analysis. Worth doing on every flight. |
| `--video <file>` | — | Fly the loop against a recording instead of the drone. |
| `--realtime` | off | With `--video`, replay at the recording's frame rate. Without it the replayed pose outruns the route and the mission times out. |
| `--dry-run` | off | Compute and print commands, never send them. |

**Speed**

| Option | Default | Meaning |
|---|---|---|
| `--cruise <n>` | `0` | **This is the speed knob.** Forward stick held on open route, on top of the PID. Measured: 40 ≈ 2 m/s. |
| `--max-speed <n>` | `25` | Cap on horizontal stick units. Raising this alone does nothing: pure pursuit pins the forward error at the lookahead distance, so the PID always asks for about `kp × lookahead` whatever the cap. Set it high (100) and control speed with `--cruise`. |
| `--min-battery <pct>` | `20` | Land below this. Refuses to take off if telemetry is missing, because an unknown battery cannot be monitored. |

**Path following**

| Option | Default | Meaning |
|---|---|---|
| `--lookahead <m>` | from mission (1.2) | How far ahead on the path to aim. Grows automatically with measured speed. |
| `--corner-limit <deg>` | `35` | Shorten the lookahead once the route bends this far, so corners are not cut through. Smaller = tighter cornering. |
| `--corner-brake <s>` | `1.5` | Start braking this long before a bend. Multiplied by measured speed, so faster flight brakes earlier. |
| `--corner-min <0-1>` | `0.25` | Floor on the cruise term in the sharpest bends. |
| `--centering <w>` | `1.0` | How hard to pull back onto the route, measured perpendicular to it. `0` is plain pure pursuit. Measured: `1.0` cut mean cross-track error from 0.43 m to 0.16 m. |
| `--yaw-gain <n>` | `90` | Yaw stick per **radian** of heading error. Easy to under-set: at 40, a 15° error asks for only 12 of 100 sticks. |
| `--max-yaw <n>` | `60` | Cap on yaw stick units. |
| `--stop-at-waypoints` | off | Settle at every waypoint instead of flying through. Slower and stuttery; kept for comparison. |

**Startup and safety**

| Option | Default | Meaning |
|---|---|---|
| `--relocalize-on-ground` | off | Bootstrap before takeoff rather than hovering. Needs someone to carry the drone around: monocular bootstrap needs translation, and a drone on the floor provides none. |
| `--bootstrap-timeout <s>` | `90` | Land if not localized in this long. |
| `--no-viewer` | off | No Pangolin window. |
| `--no-preview` | off | No OpenCV window. **Disables manual takeover and `Esc`** - only Ctrl-C still lands. |

**In-flight keys** (the preview window must have focus)

| Key | Action |
|---|---|
| `space` | Take manual control; the drone hovers and the mission is suspended. |
| `m` | Hand back to the mission, resuming at the current waypoint. |
| `w`/`s` | Forward / back |
| `a`/`d` | Left / right |
| `r`/`f` | Up / down |
| `q`/`e` | Yaw left / right |
| `Esc` | Land and quit |
| `x` | EMERGENCY STOP (cuts the motors) |

Any movement key also takes manual control, so in an emergency just push a
direction - no need to remember `space` first.

The HUD shows mode, waypoint, forward/lateral/vertical error, speed, current
lookahead, cruise percentage and heading error.

### 3.7 `calibrate_camera`

```sh
./build/apps/tools/calibrate_camera/calibrate_camera \
    --video recordings/chessboard.h264 --cols 8 --rows 10 --square 0.025 \
    --views 30 --out config/tello_camera.yaml
```

| Option | Default | Meaning |
|---|---|---|
| `--video <file>` | — | Calibrate from a recording. |
| `--drone` | — | Calibrate from the live stream (the default when `--video` is absent). |
| `--out <yaml>` | required | Config to write. |
| `--cols <n>` | `9` | **Inner corners** across - one fewer than squares. |
| `--rows <n>` | `6` | **Inner corners** down. |
| `--square <m>` | `0.025` | Square size in metres. |
| `--views <n>` | `25` | Views to collect. |
| `--min-gap <n>` | `15` | Frames to skip between accepted views, so near-duplicates do not bias the fit. |

Counting squares instead of inner corners is the usual mistake: an 11×9-square
board is `--cols 8 --rows 10`. The order of `--cols`/`--rows` does not matter.
Aim for an RMS reprojection error under 0.5 px, and make sure the board
reaches all four corners of the frame - that is what pins down the distortion.

### 3.8 `calibrate_scale`

```sh
./build/apps/tools/calibrate_scale/calibrate_scale \
    --map maps/office.osa --drone --distance 2.0 --no-viewer
```

| Option | Default | Meaning |
|---|---|---|
| `--map <map.osa>` | required | Map to calibrate. Its metadata is updated in place. |
| `--drone` | — | Localize from the live stream. |
| `--video <file>` | — | Localize from a recording instead. |
| `--ip <addr>` | `192.168.10.1` | Drone address. |
| `--distance <m>` | required | Real distance between the two marks. |
| `--set-scale <m>` | — | Write an already-measured scale and exit. No drone needed. |
| `--camera <yaml>` | `config/tello_camera.yaml` | Camera intrinsics. |
| `--no-viewer` | off | No Pangolin window. |

Keys: `a` marks point A, `b` marks point B and records a sample, `Enter`
saves, `Esc` quits without saving. Carry the drone by hand between two marks
and repeat 4-5 times; hand-carrying is both safer and more accurate than
flying the measurement. The tool warns if the samples disagree by more than
10%.

A monocular map has no absolute scale, and `autonomous_mission` refuses to fly
a map that has not been calibrated.

### 3.9 `trajectory_to_mission`

```sh
./build/apps/tools/trajectory_to_mission/trajectory_to_mission \
    --map maps/office.osa --out config/mission.yaml \
    --spacing 0.8 --skip 5 --limit 12
```

| Option | Default | Meaning |
|---|---|---|
| `--map <map.osa>` | required | Map the trajectory belongs to; supplies the scale. |
| `--trajectory <file>` | from the map's metadata | Trajectory to sample. |
| `--out <yaml>` | required | Route to write. |
| `--spacing <m>` | `0.6` | Distance between waypoints. |
| `--tolerance <m>` | `0.25` | Arrival tolerance. |
| `--free-heading` | off | Do not command a heading at each waypoint. |
| `--skip <n>` | `0` | Drop the first n waypoints - the taxi-out part of the recording. |
| `--limit <n>` | no limit | Keep at most n waypoints. **Use this for a first flight.** |
| `--margin <m>` | `3.0` | Geofence margin around the route. |
| `--name <text>` | derived | Mission name. |

Waypoints come from the trajectory the mapping run reconstructed, so every one
is a pose the camera has already localized from. The geofence is sized to the
route plus the margin rather than to a fixed default, which would either abort
a legitimate route immediately or protect nothing.

### 3.10 Test tools (`BUILD_TESTS=ON`)

```sh
./build/tests/tools/make_synthetic_video --frames 420 --seed 7
./build/tests/tools/compare_trajectory --estimate maps/x_trajectory.txt \
    --truth tests/data/synthetic_truth.txt --write-scale maps/x.osa
```

| `make_synthetic_video` | Default | Meaning |
|---|---|---|
| `--out <file.avi>` | `tests/data/synthetic.avi` | Video to write. |
| `--truth <file.txt>` | `tests/data/synthetic_truth.txt` | Ground-truth trajectory, TUM format, in metres. |
| `--camera <file.yaml>` | `tests/data/synthetic_camera.yaml` | Matching camera config. |
| `--frames <n>` | `420` | Frame count. |
| `--seed <n>` | `7` | RNG seed; the scene is deterministic. |

| `compare_trajectory` | Default | Meaning |
|---|---|---|
| `--estimate <file.txt>` | required | Trajectory from `map_builder`. |
| `--truth <file.txt>` | required | Ground truth. |
| `--tolerance <s>` | `0.02` | Timestamp match window. |
| `--write-scale <map.osa>` | — | Record the fitted scale in that map's metadata. |

Fits the best similarity transform (Umeyama) and reports scale and absolute
trajectory error.

---

## 4. Testing

```sh
cd build && ctest --output-on-failure     # unit tests, no hardware
./scripts/run_pipeline_test.sh            # the whole chain, no hardware
```

The unit tests cover the places where a bug is expensive to find in the air:
the map/body rotations, the PID at waypoint transitions and deadband edges,
path following (lookahead, corner truncation, corner braking, centering),
every safety path in the mission planner, the H264 decoder under packet loss,
and the UDP socket's timeout behaviour.

`scripts/run_pipeline_test.sh` renders a synthetic corridor flythrough with
ground truth in metres, builds a map from it, checks the reconstruction
against the truth, localizes against the saved map, derives a route and flies
it in dry-run, asserting at each step. On the reference machine it
reconstructs an 8 m route to about 1.7 mm RMS (0.02% of path length),
relocalizes in ~40 frames, and completes the route with no control spikes.

---

## 5. Configuration files

`config/tello_camera.yaml` - camera intrinsics in ORB-SLAM3's schema. Every
numeric camera parameter must be written with a decimal point: ORB-SLAM3
rejects an integer node and aborts the process with a one-line message.
`Camera.RGB` must stay `0`, because `video/` produces BGR.

`maps/<name>.meta.yaml` - written by `map_builder`, holds the scale,
calibration flag, keyframe count and provenance (which video, camera config
and vocabulary produced the map).

`config/mission_*.yaml` - written by `trajectory_to_mission`. Beyond the
waypoint list it carries `geofence_*`, `max_tracking_loss_ms`,
`min_confidence`, `continuous`, `lookahead_m`, `lookahead_time_s`,
`lookahead_max_m`, `lookahead_corner_limit_rad`, `corner_brake_time_s`,
`corner_full_slow_rad` and `corner_min_cruise_scale`. Command-line flags
override these per flight.

---

## 6. Two things that will confuse you if nobody says them

### Loading a map does not put ORB-SLAM3 into it

On loading an atlas, ORB-SLAM3 calls `Atlas::CreateNewMap()` and tracks in a
fresh empty map. `Relocalization()` only ever searches keyframes belonging to
the *active* map, so it can never find its way into the loaded one directly.
The only route in is place recognition merging the new session's map into the
pre-built one.

So localization has a bootstrap phase: the drone builds a small throwaway map,
recognizes the place, and the two are merged, at which point poses switch into
the pre-built map's frame. Bootstrapping needs **translation**, so a drone held
still never gets there - which is why `autonomous_mission` takes off first and
lets you fly it manually until it matches.

`slam::LocalizerStats::in_prebuilt_map` is the flag that says the merge has
happened, and `processFrame()` will not report `TrackingState::Ok` until it
has: before the merge the pose is perfectly valid in a coordinate system that
has nothing to do with the mission's waypoints.

**Do not use ORB-SLAM3's localization-only mode.** Freezing the map sounds
obviously right and measures catastrophically. Replaying a 623-keyframe map
against its own footage, sustained tracking was 0.3% of frames when frozen on
merge, 1.0% frozen after a delay, and **99.4% never frozen** - it cannot create
map points, so it thrashes the moment the camera strays from what the map
already covers. The map file on disk is never written during localization
either way.

### Which way is up

| Frame | Convention |
|---|---|
| Camera (ORB-SLAM3) | x right, y down, z forward |
| Map | the camera frame of the first keyframe, frozen |
| Body (`rc a b c d`) | x forward, y left, z up |

The map frame is *not* a world frame: its axes are wherever the camera pointed
when mapping began. The useful consequence is that the rotation in every SLAM
pose already says where the drone is pointing inside the map, so no takeoff
alignment calibration is needed. What SLAM cannot tell you is which way gravity
points - `FrameAlignment::map_up` defaults to map `-y`, correct when the camera
was level at the first keyframe.

---

## 7. Known issues and limitations

* **Segfault on exit with the Pangolin viewer.** `map_builder` can crash at
  process exit when the viewer is enabled - *after* the map and metadata are
  safely written, so the output is complete and usable. ORB-SLAM3's `System`
  has no destructor and never joins its three threads. Use `--no-viewer`.
* **A map only works under the lighting it was built in.** See section 2.
  `--extend` can add a session to an existing map, but only if place
  recognition matches the two, which a day/night change defeats.
* **The scale is measured once, by hand.** There is no online scale estimation.
* **Loop closure stays active during flight.** It has to: the merge that gets
  us into the pre-built map is loop closure's work. A large closure runs a
  global bundle adjustment that could shift the map frame mid-flight, taking
  the waypoints with it. Not observed in practice, and not mitigated in code.
* **Monocular tracking is fragile in the open.** Blank walls, low light and
  fast rotation all lose tracking. Keep the route where the map is dense.
* **No obstacle sensing.** The drone follows the recorded route; if the
  recording hugged a wall, so will the flight.
* Whether `rc` streaming alone resets the Tello's ~15 s no-activity auto-land
  timer is still unverified; `TelloDriver` sends a keep-alive regardless.

---

## 8. Layout

```
common/         shared types, logging, queues, UDP socket - no external deps
core/           EventBus (typed pub/sub) + module lifecycle
drivers/tello/  Tello SDK 2.0 UDP protocol, behind the IDrone interface
video/          H264 decode (libavcodec); live-drone and video-file sources
slam/           ORB-SLAM3 wrapper behind ILocalizer; map metadata; camera config
control/        waypoints, mission planner, PID + path following, frame alignment
apps/           one executable per milestone, plus tools/
third_party/    ORB-SLAM3 and Pangolin submodules, built from our CMake
tests/          unit tests and the synthetic-data tooling
scripts/        vocabulary extraction, end-to-end pipeline test
```

Modules talk through `core::EventBus` topics carrying plain types from
`common/types.hpp`. `control/` depends on neither `slam/` nor `drivers/` - it
consumes plain poses and emits plain commands, which is what lets every safety
path be unit-tested with no hardware and no SLAM backend.

### How ORB-SLAM3 is built here

`third_party/CMakeLists.txt` compiles ORB-SLAM3, DBoW2 and g2o from *our*
CMake and never invokes upstream's, which declares
`cmake_minimum_required(VERSION 2.8)` (rejected by CMake 4), appends
`-march=native -O3 -std=c++11` to the **global** flags, and builds into its own
source tree.

The submodule is used almost unpatched, at upstream `4452a3c`:

* the vendored code is built as **C++14**, because `LoopClosing.cc` increments
  a `bool`, which C++17 removed. `slam/` links it `PRIVATE` and exposes no
  ORB-SLAM3 header, so that standard never leaks into the rest of the project;
* g2o's headers reach for `#include "../../config.h"`, which no `-I` can
  satisfy; mirroring g2o's directory depth in the build tree does;
* **one source-level fix**, applied to a *copy* of `Map.cc` in the build tree
  so the checkout stays pristine: `Map::PreSave()` walks `mspMapPoints` with a
  range-for while the body can erase from that same set
  (`EraseObservation` → `SetBadFlag` → `Map::EraseMapPoint`), so saving any map
  built from a real flight crashed in `std::_Rb_tree_increment`. Configuring
  fails loudly if upstream ever changes those lines.

Verified on Ubuntu with GCC 15.2, CMake 4.2, OpenCV 4.10, FFmpeg 8 and
Pangolin 0.9.2.

---

## 9. Safety

Built into `apps/autonomous_mission` and `control/mission_planner.cpp`:

* it will not take off until telemetry is arriving and the battery is known;
* it will not start the mission until it has relocalized **in the pre-built
  map**, and the bootstrap hover is flown manually, not autonomously;
* it will not fly a map whose scale has not been calibrated, or a route
  authored against a different map;
* any pose that is not confidently tracked commands a hover, never a guess,
  and a hover longer than `max_tracking_loss_ms` aborts;
* a geofence sized to the route, a battery floor, and a 2-second video
  watchdog abort independently of the mission logic;
* manual takeover is always one keypress away, and any direction key takes it;
* every exit path lands: completion, abort, `Esc`, and Ctrl-C.

Start a new route with `--limit` on a handful of waypoints and a low
`--cruise`, and raise both only once the loop is proven.
