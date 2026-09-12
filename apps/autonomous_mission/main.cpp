// Milestones (e)+(f): fly a pre-authored route using nothing but ORB-SLAM3
// poses against a pre-built map.
//
// The safety posture here is deliberate and not negotiable in code review:
//   * the drone does not take off until it has relocalized in the map,
//   * every pose that is not confidently tracked commands a hover, not a
//     guess,
//   * a geofence and a battery floor can abort independently of the mission
//     logic,
//   * every exit path - normal, aborted, exception, Ctrl-C - lands.
#include <algorithm>
#include <cmath>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "app_support.hpp"
#include "common/cli.hpp"
#include "common/logging.hpp"
#include "control/mission_io.hpp"
#include "control/mission_planner.hpp"
#include "control/pid_flight_controller.hpp"
#include "core/event_bus.hpp"
#include "slam/map_metadata.hpp"
#include "slam/orb_slam3_localizer.hpp"

namespace {

std::atomic<bool> g_running{true};
void handleSigint(int) { g_running = false; }

void printUsage() {
    std::cout <<
        "Fly a waypoint route closed-loop on SLAM pose feedback.\n\n"
        "  autonomous_mission --map <map.osa> --mission <mission.yaml> [options]\n\n"
        "  --map <map.osa>       map to localize against\n"
        "  --mission <yaml>      route (see tools/trajectory_to_mission)\n"
        "  --camera <yaml>       camera intrinsics  [config/tello_camera.yaml]\n"
        "  --vocabulary <file>   ORB vocabulary     [third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt]\n"
        "  --ip <addr>           drone address      [192.168.10.1]\n"
        "  --record <file>       save the live H264 stream for post-flight analysis\n"
        "  --video <file>        fly the loop against a recording instead of the drone\n"
        "  --realtime            (with --video) replay at the recording's own frame rate\n"
        "  --dry-run             compute and print commands but never send them\n"
        "  --min-battery <pct>   land below this     [20]\n"
        "  --max-speed <n>       cap on horizontal stick units [25]\n"
        "  --cruise <n>          forward stick held on open route [0 = pure PID]\n"
        "                        This, not --max-speed, is the speed knob: pure pursuit\n"
        "                        pins the forward error at the lookahead distance, so the\n"
        "                        PID alone always commands about kp x lookahead no matter\n"
        "                        how high the cap is.\n"
        "  --lookahead <m>       how far ahead on the path to aim  [from the mission]\n"
        "  --corner-limit <deg>  shorten the lookahead once the route bends this far [35]\n"
        "  --corner-brake <s>    start braking this long before a bend  [1.5]\n"
        "  --corner-min <0-1>    slowest the cruise term goes in a bend  [0.25]\n"
        "  --centering <w>       how hard to pull back onto the route  [1.0, 0 = off]\n"
        "  --yaw-gain <n>        yaw stick per radian of heading error  [90]\n"
        "  --max-yaw <n>         cap on yaw stick units  [60]\n"
        "  --stop-at-waypoints   settle at every waypoint instead of flying through\n"
        "  --relocalize-on-ground  bootstrap before takeoff instead of hovering\n"
        "                          (needs someone to carry the drone around)\n"
        "  --bootstrap-timeout <s>  land if not localized in this long  [90]\n"
        "  --no-viewer           run without the Pangolin window\n"
        "  --no-preview          run without the OpenCV camera window (headless)\n\n"
        "\nIn the preview window:\n"
        "  space = take manual control (hover)   m = hand back to the mission\n"
        "  w/s fwd-back  a/d left-right  r/f up-down  q/e yaw\n"
        "  Esc = land and quit                   x = EMERGENCY STOP (motors off)\n";
}

const char* statusName(control::MissionStatus status) {
    switch (status) {
        case control::MissionStatus::Idle: return "IDLE";
        case control::MissionStatus::Running: return "RUNNING";
        case control::MissionStatus::Holding: return "HOLDING";
        case control::MissionStatus::Complete: return "COMPLETE";
        case control::MissionStatus::Aborted: return "ABORTED";
    }
    return "?";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || argc == 1) {
        printUsage();
        return argc == 1 ? 1 : 0;
    }

    const std::string mapPath = args.get("map");
    const std::string missionPath = args.get("mission");
    if (mapPath.empty() || missionPath.empty()) {
        std::cerr << "error: --map and --mission are both required\n\n";
        printUsage();
        return 1;
    }

    const bool dryRun = args.has("dry-run");
    const int minBattery = args.getInt("min-battery", 20);
    const int bootstrapTimeoutS = args.getInt("bootstrap-timeout", 90);
    const int maxSpeed = args.getInt("max-speed", 25);

    control::Mission mission;
    if (!control::loadMission(missionPath, mission)) return 1;
    if (args.has("stop-at-waypoints")) mission.config.continuous = false;
    if (const double lookahead = args.getDouble("lookahead", 0.0); lookahead > 0.0) {
        mission.config.lookahead_m = static_cast<float>(lookahead);
    }
    if (const double brake = args.getDouble("corner-brake", 0.0); brake > 0.0) {
        mission.config.corner_brake_time_s = static_cast<float>(brake);
    }
    if (const double minScale = args.getDouble("corner-min", 0.0); minScale > 0.0) {
        mission.config.corner_min_cruise_scale = static_cast<float>(minScale);
    }
    if (const double cornerDeg = args.getDouble("corner-limit", 0.0); cornerDeg > 0.0) {
        mission.config.lookahead_corner_limit_rad =
            static_cast<float>(cornerDeg * M_PI / 180.0);
    }
    if (!mission.map_path.empty() && mission.map_path != mapPath) {
        common::logError("mission",
                          "this route was authored against '" + mission.map_path +
                              "' but --map is '" + mapPath +
                              "'. Waypoint coordinates only mean something in their own map.");
        return 1;
    }

    slam::MapMetadata metadata;
    const bool haveMetadata = slam::loadMapMetadata(mapPath, metadata);
    if (!haveMetadata || !metadata.scale_calibrated) {
        common::logError("mission",
                          "this map's scale is not calibrated, so waypoint distances are not "
                          "metres. Run tools/calibrate_scale first, or the drone will fly the "
                          "wrong distances.");
        return 1;
    }

    const std::string cameraPath = args.get("camera", "config/tello_camera.yaml");
    slam::CameraConfigInfo camera;
    if (!slam::readCameraConfig(cameraPath, camera)) return 1;

    core::EventBus bus;

    std::atomic<int> battery{-1};
    bus.subscribe<common::DroneState>(
        [&battery](const common::DroneState& state) { battery.store(state.battery_pct); });

    app::FrameFeed feed = app::openFrameFeed(args, camera, bus);
    if (!feed.source) return 1;

    const bool live = feed.live && !dryRun;

    slam::LocalizerConfig localizerConfig;
    localizerConfig.mode = slam::LocalizerMode::Localization;
    localizerConfig.vocabulary_path =
        args.get("vocabulary", "third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    localizerConfig.camera_config_path = cameraPath;
    localizerConfig.map_path = mapPath;
    localizerConfig.enable_viewer = !args.has("no-viewer");

    slam::OrbSlam3Localizer localizer;
    if (!localizer.initialize(localizerConfig)) {
        if (feed.drone) feed.drone->disconnect();
        return 1;
    }

    control::FlightControllerConfig controllerConfig;
    controllerConfig.max_horizontal_command = maxSpeed;
    controllerConfig.cruise_command = args.getInt("cruise", 0);
    controllerConfig.path_centering =
        static_cast<float>(args.getDouble("centering", 1.0));
    if (const double yawGain = args.getDouble("yaw-gain", 0.0); yawGain > 0.0) {
        controllerConfig.yaw.kp = static_cast<float>(yawGain);
    }
    controllerConfig.max_yaw_command = args.getInt("max-yaw", controllerConfig.max_yaw_command);
    control::PidFlightController controller(controllerConfig);

    control::MissionPlanner planner;
    planner.setAlignment(controllerConfig.alignment);
    planner.loadMission(mission);

    std::signal(SIGINT, handleSigint);
    const bool preview = !args.has("no-preview");

    const auto show = [&preview](const cv::Mat& image) {
        if (!preview) return -1;
        cv::imshow("autonomous_mission", image);
        return cv::waitKey(1);
    };

    // ---- Pre-flight checks, before the props ever spin. -------------------
    const auto shutdown = [&](int code) {
        if (feed.drone) {
            feed.drone->sendVelocity({});
            feed.drone->land();
            feed.drone->enableVideoStream(false);
            feed.drone->disconnect();
        }
        localizer.shutdown();
        feed.source->close();
        cv::destroyAllWindows();
        return code;
    };

    if (live) {
        // An UNKNOWN battery is a refusal, not a free pass: `battery` stays at
        // -1 until a state packet arrives on UDP 8890, so treating "unknown"
        // as "fine" would take off blind and fly with no battery monitoring.
        for (int waited = 0; battery.load() < 0 && waited < 30; ++waited) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (battery.load() < 0) {
            common::logError("mission",
                              "no telemetry from the drone (nothing on UDP 8890 in 3s), so the "
                              "battery is unknown and cannot be monitored in flight - not taking "
                              "off. Check the firewall allows inbound UDP 8890.");
            return shutdown(1);
        }
        if (battery.load() < minBattery) {
            common::logError("mission", "battery is " + std::to_string(battery.load()) +
                                             "%, below the " + std::to_string(minBattery) +
                                             "% floor - not taking off");
            return shutdown(1);
        }
        common::logInfo("mission", "battery " + std::to_string(battery.load()) + "%");

        if (!preview) {
            common::logWarn("mission",
                             "--no-preview means there is no window to receive keys, so manual "
                             "takeover and Esc-to-land are NOT available. Ctrl-C still lands.");
        }
    }

    // ---- Takeoff, then bootstrap in the air. ------------------------------
    //
    // Relocalizing BEFORE takeoff is the safer-sounding order and does not
    // work in practice: ORB-SLAM3 has to bootstrap a map of its own before it
    // can recognise the pre-built one, and bootstrapping a monocular map
    // needs translation. A drone sitting on the floor provides none, so the
    // old ground-first order just waited forever unless somebody picked the
    // drone up and carried it. Hovering gives the pilot a way to supply that
    // motion - and the bootstrap hover is flown manually, not autonomously,
    // precisely because there is no trustworthy pose to fly on yet.
    if (live && !args.has("relocalize-on-ground")) {
        std::cout << "\nTaking off in 3 seconds. Ctrl-C to abort.\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));
        if (!g_running) return shutdown(1);
        if (!feed.drone->takeoff()) {
            common::logError("mission", "takeoff was not acknowledged");
            return shutdown(1);
        }
        std::this_thread::sleep_for(std::chrono::seconds(4));  // let it stabilize
    } else if (!live) {
        std::cout << (dryRun ? "\nDRY RUN: commands will be printed, not sent.\n"
                              : "\nNo drone: running the loop against the video only.\n");
        if (!args.get("video").empty() && !args.has("realtime")) {
            // Without pacing, the replayed pose sprints along the recorded
            // path while the waypoints advance at their own rate, so the
            // controller is chasing a target tens of metres behind and the
            // mission times out. That looks like a controller bug and is not.
            common::logWarn("mission",
                             "replaying a recording at full decode speed - the pose will outrun "
                             "the route and the mission will time out. Add --realtime for a "
                             "dry run whose errors mean anything.");
        }
    }

    std::cout << "\n" << app::ManualPilot::keyHelp() << "\n"
              << "space = hover + take manual control     m = hand back to the mission\n"
              << "Esc = land and quit                     x = EMERGENCY STOP (motors off)\n\n"
              << "Waiting to relocalize. Fly gently sideways - monocular SLAM needs\n"
              << "translation to bootstrap, and cannot recognise the map until it has.\n\n";

    enum class Mode { Bootstrapping, Auto, Manual };
    Mode mode = Mode::Bootstrapping;

    app::ManualPilot pilot(std::min(maxSpeed, 25));
    auto previousAt = std::chrono::steady_clock::now();
    auto lastFrameAt = previousAt;
    const auto bootstrapStartedAt = previousAt;
    bool hintedBootstrap = false;
    auto lastBootstrapReportAt = previousAt;

    const auto modeName = [](Mode value) {
        switch (value) {
            case Mode::Bootstrapping: return "BOOTSTRAP";
            case Mode::Auto: return "AUTO";
            case Mode::Manual: return "MANUAL";
        }
        return "?";
    };

    while (g_running) {
        auto frame = feed.source->nextFrame(500);
        const auto now = std::chrono::steady_clock::now();

        if (!frame) {
            // A stalled feed is as dangerous as lost tracking, and the planner
            // cannot see it: it simply never gets a pose.
            if (now - lastFrameAt > std::chrono::seconds(2)) {
                if (mode == Mode::Manual) {
                    // The pilot is flying; losing video is not a reason to
                    // take the aircraft away from them.
                    common::logWarn("mission", "no video, but you have manual control");
                } else {
                    planner.abort("no video frames for 2 seconds");
                    break;
                }
            }
            if (live) feed.drone->sendVelocity(pilot.velocity(now));
            if (preview) {
                const int key = app::showVideoStalled("autonomous_mission",
                                                       feed.source->healthSummary(),
                                                       "Esc = land and quit");
                if ((key & 0xFF) == 27) break;
            }
            if (!feed.source->isOpen()) break;
            continue;
        }
        lastFrameAt = now;

        const common::PoseEstimate pose = localizer.processFrame(*frame);
        const auto stats = localizer.stats();

        if (live && battery.load() >= 0 && battery.load() < minBattery) {
            common::logError("mission", "battery below " + std::to_string(minBattery) +
                                             "% - landing");
            break;
        }

        // ---- Mode transitions driven by tracking state -------------------
        if (mode == Mode::Bootstrapping && stats.in_prebuilt_map &&
            pose.state == common::TrackingState::Ok &&
            pose.confidence >= mission.config.min_confidence) {
            std::cout << "\nRelocalized in the map - starting the mission.\n";
            mode = Mode::Auto;
            pilot.stop();
            controller.reset();
            planner.start();
            previousAt = now;
        }

        if (mode == Mode::Bootstrapping &&
            now - bootstrapStartedAt > std::chrono::seconds(bootstrapTimeoutS)) {
            common::logError("mission", "did not relocalize within " +
                                             std::to_string(bootstrapTimeoutS) + "s - landing");
            break;
        }
        // A bootstrap that is not converging needs numbers, not adjectives:
        // "no map yet" and "a map that will not match" look identical from
        // the outside and have completely different fixes.
        if (mode == Mode::Bootstrapping && now - lastBootstrapReportAt > std::chrono::seconds(5)) {
            lastBootstrapReportAt = now;
            const double seconds =
                std::chrono::duration<double>(now - bootstrapStartedAt).count();
            std::ostringstream report;
            report << std::fixed << std::setprecision(1) << seconds << "s: "
                   << app::trackingStateName(pose.state) << ", own map "
                   << stats.map_keyframes << " keyframes, " << stats.tracked_map_points
                   << " points tracked, "
                   << (stats.frames_processed > 0
                           ? static_cast<double>(stats.frames_processed) / std::max(seconds, 0.1)
                           : 0.0)
                   << " fps";
            const std::string health = feed.source->healthSummary();
            if (!health.empty()) report << " | " << health;
            common::logInfo("bootstrap", report.str());
        }
        if (mode == Mode::Bootstrapping && !hintedBootstrap &&
            now - bootstrapStartedAt > std::chrono::seconds(20)) {
            hintedBootstrap = true;
            common::logWarn("mission",
                             stats.map_keyframes > 0
                                 ? "tracking its own map but no match with the pre-built one yet "
                                   "- fly towards somewhere distinctive that the mapping run "
                                   "definitely covered, and consider whether the lighting has "
                                   "changed since the map was built"
                                 : "no map bootstrapped yet - fly gently sideways, and make sure "
                                   "the camera sees texture rather than a blank wall");
        }

        // ---- Decide the command for this frame ---------------------------
        common::VelocityCommand command;
        std::string hud = modeName(mode);

        if (mode == Mode::Auto) {
            planner.onPoseUpdate(pose, now);
            const auto status = planner.status();
            hud += std::string("  ") + statusName(status) + "  wp " +
                   std::to_string(planner.currentIndex() + 1) + "/" +
                   std::to_string(planner.waypointCount());

            if (status == control::MissionStatus::Complete ||
                status == control::MissionStatus::Aborted) {
                break;
            }
            if (status == control::MissionStatus::Running) {
                if (const auto target = planner.currentTarget()) {
                    const std::chrono::duration<double> dt = now - previousAt;
                    controller.setCruiseScale(planner.cruiseScale());
                    controller.setPathOffset(planner.pathOffset());
                    command = controller.computeCommand(pose, *target, dt);
                    const auto error = controller.lastBodyError();
                    std::ostringstream errorText;
                    errorText << std::fixed << std::setprecision(2) << "err fwd " << error.x
                              << " lat " << error.y << " vert " << error.z << "   "
                              << std::setprecision(1) << planner.speed() << " m/s  look "
                              << std::setprecision(2) << planner.lookahead() << " m  cruise "
                              << std::setprecision(0) << planner.cruiseScale() * 100.0f << "%"
                              << "  hdg " << std::setprecision(0)
                              << controller.lastHeadingError() * 180.0f / static_cast<float>(M_PI)
                              << " deg";
                    hud += "   " + errorText.str();
                }
            } else {
                controller.reset();  // Holding: hover without winding up
            }
        } else {
            command = pilot.velocity(now);
            if (mode == Mode::Bootstrapping) {
                hud += stats.in_prebuilt_map ? "  matched, waiting for a confident pose"
                                              : "  looking for the map";
            }
        }
        previousAt = now;

        if (live) {
            feed.drone->sendVelocity(command);
        } else if (dryRun) {
            std::cout << "\r  rc " << command.roll << " " << command.pitch << " "
                      << command.throttle << " " << command.yaw << "   " << hud << "      "
                      << std::flush;
        }

        // ---- Input -------------------------------------------------------
        cv::Mat display = app::drawPoseOverlay(frame->image, pose, hud);
        cv::putText(display, app::ManualPilot::keyHelp(), {12, display.rows - 32},
                     cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(190, 190, 190), 1);
        cv::putText(display, "space=take over   m=resume mission   Esc=land   x=E-STOP",
                     {12, display.rows - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                     cv::Scalar(190, 190, 190), 1);
        const int rawKey = show(display);
        const int key = rawKey < 0 ? -1 : (rawKey & 0xFF);

        if (key == 27) {  // Esc: land now
            planner.abort("operator pressed Esc");
            break;
        }
        if (key == 'x') {  // last resort: cut the motors
            common::logError("mission", "EMERGENCY STOP");
            if (live) feed.drone->emergencyStop();
            g_running = false;
            break;
        }
        if (key == ' ') {
            // One key that always means "stop what you are doing". Zeroing
            // the axes too, so taking over never inherits a stale velocity.
            pilot.stop();
            if (mode != Mode::Manual) {
                mode = Mode::Manual;
                common::logWarn("mission", "MANUAL control - the mission is suspended");
            }
        } else if (key == 'm') {
            if (mode == Mode::Manual) {
                if (stats.in_prebuilt_map) {
                    mode = Mode::Auto;
                    pilot.stop();
                    controller.reset();
                    planner.resume();
                } else {
                    common::logWarn("mission",
                                     "not localized in the map - staying in manual control");
                }
            } else if (mode == Mode::Auto) {
                mode = Mode::Manual;
                pilot.stop();
                common::logWarn("mission", "MANUAL control - the mission is suspended");
            }
        } else if (key >= 0 && pilot.onKey(key, now)) {
            if (mode == Mode::Auto) {
                mode = Mode::Manual;
                common::logWarn("mission", "MANUAL control - the mission is suspended");
            }
        }
    }

    std::cout << "\n\nMission " << statusName(planner.status());
    if (!planner.abortReason().empty()) std::cout << ": " << planner.abortReason();
    std::cout << "\nReached " << planner.currentIndex() << " of " << planner.waypointCount()
              << " waypoints.\n";

    return shutdown(planner.status() == control::MissionStatus::Complete ? 0 : 1);
}
