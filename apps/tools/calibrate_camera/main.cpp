// Chessboard calibration for the Tello's front camera, writing the result in
// the schema ORB-SLAM3 expects.
//
// This is not optional polish. ORB-SLAM3 triangulates from the intrinsics it
// is given; wrong ones bend the reconstruction, and the symptom is not an
// error message but a map that quietly fails to relocalize later. Calibrate
// once per airframe.
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "app_support.hpp"
#include "common/cli.hpp"
#include "common/logging.hpp"
#include "core/event_bus.hpp"
#include "slam/camera_config.hpp"

namespace {

volatile std::sig_atomic_t g_interrupted = 0;
void handleSigint(int) { g_interrupted = 1; }

void printUsage() {
    std::cout <<
        "Calibrate the camera from chessboard views.\n\n"
        "  calibrate_camera --video recordings/chessboard.h264 --out config/tello_camera.yaml\n"
        "  calibrate_camera --drone --out config/tello_camera.yaml\n\n"
        "  --video <file> | --drone   where the images come from\n"
        "  --out <yaml>               config to write\n"
        "  --cols <n>                 inner corners across  [9]\n"
        "  --rows <n>                 inner corners down    [6]\n"
        "  --square <m>               square size in metres [0.025]\n"
        "  --views <n>                views to collect      [25]\n"
        "  --min-gap <n>              frames to skip between accepted views [15]\n\n"
        "Hold a printed chessboard at many angles and distances, filling different\n"
        "parts of the frame - corner coverage is what pins down the distortion.\n";
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help") || argc == 1) {
        printUsage();
        return argc == 1 ? 1 : 0;
    }

    const std::string outPath = args.get("out");
    if (outPath.empty()) {
        std::cerr << "error: --out is required\n\n";
        printUsage();
        return 1;
    }

    const cv::Size boardSize(args.getInt("cols", 9), args.getInt("rows", 6));
    const float squareSize = static_cast<float>(args.getDouble("square", 0.025));
    const std::size_t wantedViews = static_cast<std::size_t>(args.getInt("views", 25));
    const int minGap = args.getInt("min-gap", 15);

    // Calibration must see the camera's NATIVE frames - resizing them would
    // change the very intrinsics we are trying to measure. So no target size
    // is imposed here, unlike everywhere else in this project.
    slam::CameraConfigInfo passthrough;
    core::EventBus bus;
    app::FrameFeed feed = app::openFrameFeed(args, passthrough, bus);
    if (!feed.source) return 1;

    const auto buildBoardCorners = [squareSize](const cv::Size& size) {
        std::vector<cv::Point3f> corners;
        for (int row = 0; row < size.height; ++row) {
            for (int col = 0; col < size.width; ++col) {
                corners.emplace_back(static_cast<float>(col) * squareSize,
                                      static_cast<float>(row) * squareSize, 0.0f);
            }
        }
        return corners;
    };
    
    // In practice findChessboardCorners copes with --cols/--rows being given
    // the wrong way round (it matches the grid it finds in either
    // orientation), so this fallback is a cheap backstop rather than the
    // usual path - but it costs one extra attempt only while no board has
    // been found yet, and it keeps the tool working if that tolerance ever
    // changes. What it does NOT rescue is counting squares instead of inner
    // corners, which no amount of transposing fixes.
    cv::Size activeSize = boardSize;
    const cv::Size transposed(boardSize.height, boardSize.width);
    std::vector<cv::Point3f> boardCorners = buildBoardCorners(activeSize);
    bool sizeLocked = false;

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;
    int sinceAccepted = minGap;

    std::signal(SIGINT, handleSigint);
    std::cout << "\nCollecting " << wantedViews << " views of a " << boardSize.width << "x"
              << boardSize.height << " INNER-CORNER board (" << squareSize << " m squares).\n"
              << "Inner corners are one fewer than squares in each direction: an 11x9\n"
              << "square board is --cols 8 --rows 10.\n"
              << "Esc to stop early.\n\n";

    const auto searchStartedAt = std::chrono::steady_clock::now();
    bool hinted = false;

    while (!g_interrupted && imagePoints.size() < wantedViews) {
        auto frame = feed.source->nextFrame(1000);
        if (!frame) {
            if (!feed.source->isOpen()) break;
            continue;
        }
        imageSize = frame->image.size();
        ++sinceAccepted;

        cv::Mat gray;
        cv::cvtColor(frame->image, gray, cv::COLOR_BGR2GRAY);

        constexpr int kDetectFlags =
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, activeSize, corners, kDetectFlags);
        if (!found && !sizeLocked && transposed != activeSize) {
            found = cv::findChessboardCorners(gray, transposed, corners, kDetectFlags);
            if (found) {
                activeSize = transposed;
                boardCorners = buildBoardCorners(activeSize);
                std::cout << "  (board is " << activeSize.width << "x" << activeSize.height
                          << " inner corners, not " << boardSize.width << "x" << boardSize.height
                          << " - using the transposed size)\n";
            }
        }
        if (found) sizeLocked = true;

        cv::Mat display = frame->image.clone();
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                                               0.01));
            cv::drawChessboardCorners(display, activeSize, corners, found);

            // Consecutive frames of a hand-held board are near-duplicates and
            // add nothing but bias, so views are spaced out.
            if (sinceAccepted >= minGap) {
                imagePoints.push_back(corners);
                objectPoints.push_back(boardCorners);
                sinceAccepted = 0;
                std::cout << "  view " << imagePoints.size() << "/" << wantedViews << "\n";
            }
        }

        cv::putText(display,
                     std::to_string(imagePoints.size()) + "/" + std::to_string(wantedViews) +
                         (found ? "  board found" : "  looking for the board"),
                     {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                     found ? cv::Scalar(80, 220, 80) : cv::Scalar(200, 200, 200), 2);
        cv::imshow("calibrate_camera", display);
        if (cv::waitKey(1) == 27) break;

        if (!hinted && !sizeLocked &&
            std::chrono::steady_clock::now() - searchStartedAt > std::chrono::seconds(15)) {
            hinted = true;
            common::logWarn("calibrate_camera",
                             "no board detected in 15s. --cols/--rows count INNER CORNERS (one "
                             "fewer than squares per side), so an 11x9-square board is "
                             "--cols 8 --rows 10. Also make sure the whole board is in frame, "
                             "lit evenly, and flat.");
        }
    }

    feed.source->close();
    if (feed.drone) {
        feed.drone->enableVideoStream(false);
        feed.drone->disconnect();
    }
    cv::destroyAllWindows();

    if (imagePoints.size() < 5) {
        common::logError("calibrate_camera",
                          "only " + std::to_string(imagePoints.size()) +
                              " usable views - need at least 5, and 15+ for a good result");
        return 1;
    }

    cv::Mat cameraMatrix;
    cv::Mat distortion;
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;
    const double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                            distortion, rotations, translations);

    std::cout << "\nRMS reprojection error: " << std::fixed << std::setprecision(3) << rms
              << " px\n";
    if (rms > 1.0) {
        std::cout << "That is high. Under ~0.5 px is a good calibration; re-shoot with the\n"
                     "board filling more of the frame and covering the corners.\n";
    }

    slam::CameraConfigInfo info;
    info.width = imageSize.width;
    info.height = imageSize.height;
    info.fps = 30;
    info.fx = cameraMatrix.at<double>(0, 0);
    info.fy = cameraMatrix.at<double>(1, 1);
    info.cx = cameraMatrix.at<double>(0, 2);
    info.cy = cameraMatrix.at<double>(1, 2);

    // OpenCV orders them k1 k2 p1 p2 k3, which is the order CameraConfigInfo
    // and the ORB-SLAM3 schema both use.
    double coefficients[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < std::min(5, distortion.cols * distortion.rows); ++i) {
        coefficients[i] = distortion.at<double>(i);
    }
    info.k1 = coefficients[0];
    info.k2 = coefficients[1];
    info.p1 = coefficients[2];
    info.p2 = coefficients[3];
    info.k3 = coefficients[4];
    if (!slam::writeCameraConfig(outPath, info)) return 1;

    std::cout << "\nfx " << info.fx << "  fy " << info.fy << "  cx " << info.cx << "  cy "
              << info.cy << "  at " << info.width << "x" << info.height << "\n"
              << "Written to " << outPath << "\n\n";
    return 0;
}
