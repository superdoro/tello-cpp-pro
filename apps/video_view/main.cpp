// Milestone (b): decode and display the Tello's H264 stream, and record it.
//
// The recording is what map_builder consumes, so this is step one of the
// whole workflow: fly the route by hand with `--record`, then build the map
// from the file it leaves behind.
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common/cli.hpp"
#include "common/logging.hpp"
#include "core/event_bus.hpp"
#include "drivers/tello/tello_driver.hpp"
#include "video/tello_video_source.hpp"

namespace {

volatile std::sig_atomic_t g_interrupted = 0;
void handleSigint(int) { g_interrupted = 1; }

void printUsage() {
    std::cout <<
        "View (and optionally record) the Tello video stream.\n\n"
        "  video_view [--ip <addr>] [--record recordings/route.h264]\n\n"
        "  --ip <addr>      drone address  [192.168.10.1]\n"
        "  --record <file>  save the raw H264 stream for offline map building\n"
        "  --width/--height resize frames before display/record  [native]\n\n"
        "  Esc or Ctrl-C to quit.\n";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help")) {
        printUsage();
        return 0;
    }

    core::EventBus bus;
    drivers::tello::TelloConnectionConfig connection;
    connection.ip = args.get("ip", "192.168.10.1");

    drivers::tello::TelloDriver drone(connection, bus);
    if (!drone.connect()) return 1;

    video::TelloVideoConfig videoConfig;
    videoConfig.record_path = args.get("record");
    videoConfig.target_width = args.getInt("width", 0);
    videoConfig.target_height = args.getInt("height", 0);

    // Bind the video port BEFORE asking the drone to stream. If `streamon`
    // goes first, the drone's first packets arrive at a port nobody is
    // listening on, the kernel answers with ICMP port-unreachable, and some
    // Tello firmware simply stops sending after that - leaving a connection
    // that looks fine and a video stream that never appears.
    video::TelloVideoSource source(videoConfig);
    if (!source.open()) {
        drone.disconnect();
        return 1;
    }
    if (!drone.enableVideoStream(true)) {
        source.close();
        drone.disconnect();
        return 1;
    }

    std::signal(SIGINT, handleSigint);
    std::cout << "\nStreaming. Esc to quit."
              << (videoConfig.record_path.empty()
                       ? "\n\n"
                       : "\nRecording to " + videoConfig.record_path + "\n\n");

    int battery = -1;
    int framesThisSecond = 0;
    int displayedFps = 0;
    auto lastRateAt = std::chrono::steady_clock::now();
    bus.subscribe<common::DroneState>(
        [&battery](const common::DroneState& state) { battery = state.battery_pct; });

    while (!g_interrupted) {
        auto frame = source.nextFrame(1000);
        if (!frame) {
            // Say WHICH half of the path is broken, rather than just "no
            // video": the fix for a silent link is nothing like the fix for
            // a stream that arrives but will not decode.
            const std::string link = std::to_string(source.receivedDatagrams()) +
                                      " datagrams / " +
                                      std::to_string(source.receivedBytes() / 1024) + " KiB, " +
                                      std::to_string(source.decodedCount()) + " frames decoded, " +
                                      std::to_string(source.decoderErrors()) + " decoder errors";
            if (source.receivedDatagrams() == 0) {
                common::logWarn("video_view",
                                 "no UDP datagrams on port " +
                                     std::to_string(videoConfig.port) +
                                     " at all - the drone is not sending, or something is "
                                     "dropping the packets (firewall? another process bound "
                                     "to that port? wrong network interface?)");
            } else if (source.decodedCount() == 0) {
                common::logWarn("video_view",
                                 "data is arriving but nothing has decoded yet (" + link +
                                     ") - normally this clears as soon as the drone sends its "
                                     "next keyframe; if it persists the stream is too lossy to "
                                     "decode");
            } else {
                // Frames HAVE decoded, so the link and the decoder both work;
                // this is a gap, not a failure.
                common::logWarn("video_view", "no new frame in the last second (" + link + ")");
            }
            continue;
        }

        cv::Mat display = frame->image.clone();
        // Measured over the last second, so a stream that is technically
        // alive but delivering 3fps is visibly different from a healthy one.
        ++framesThisSecond;
        const auto now = std::chrono::steady_clock::now();
        if (now - lastRateAt >= std::chrono::seconds(1)) {
            displayedFps = framesThisSecond;
            framesThisSecond = 0;
            lastRateAt = now;
        }

        std::ostringstream hud;
        hud << frame->image.cols << "x" << frame->image.rows << "  " << displayedFps << " fps"
            << "  frame " << frame->index;
        if (battery >= 0) hud << "  battery " << battery << "%";
        if (source.decoderErrors() > 0) hud << "  decode err " << source.decoderErrors();
        if (source.droppedCount() > 0) hud << "  dropped " << source.droppedCount();
        cv::putText(display, hud.str(), {12, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    battery >= 0 && battery < 20 ? cv::Scalar(60, 60, 230)
                                                  : cv::Scalar(220, 220, 220),
                    1);

        cv::imshow("Tello video", display);
        if (cv::waitKey(1) == 27) break;
    }

    std::cout << "\nLink:    " << source.receivedDatagrams() << " datagrams, "
              << source.receivedBytes() / 1024 << " KiB\n"
              << "Decoded: " << source.decodedCount() << " frames, "
              << source.decoderErrors() << " decoder errors\n"
              << "Dropped: " << source.droppedCount() << " frames (consumer too slow)\n"
              << "\nSet TELLO_FFMPEG_LOG=1 to see libavcodec's own decoding complaints.\n";

    source.close();
    drone.enableVideoStream(false);
    drone.disconnect();
    return 0;
}
