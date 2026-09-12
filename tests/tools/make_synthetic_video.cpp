// Renders a synthetic flight through a textured corridor, with ground truth.
//
// Real end-to-end testing of a SLAM pipeline needs footage, and footage needs
// a drone, a room and a battery. This generates a substitute with three
// properties a real recording cannot offer: it is deterministic, it comes
// with the exact camera trajectory in METRES, and its intrinsics are known
// perfectly. That makes it possible to check not just "does the pipeline
// run" but "is the scale it recovers right".
//
// The scene is a corridor lined with small textured planar patches at
// assorted depths and orientations. Patches rather than flat walls, because
// monocular initialization degenerates on a single plane - the depth spread
// is what gives ORB-SLAM3 the parallax it needs.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "common/cli.hpp"

namespace {

struct Patch {
    cv::Point3d centre;
    cv::Point3d right;  // half-extent along the patch's local x
    cv::Point3d up;     // half-extent along the patch's local y
    cv::Mat texture;
};

cv::Mat makeTexture(std::mt19937& rng, int size) {
    cv::Mat texture(size, size, CV_8UC3);
    std::uniform_int_distribution<int> colour(0, 255);
    std::uniform_int_distribution<int> coordinate(0, size - 1);
    std::uniform_int_distribution<int> radius(size / 12, size / 4);

    texture.setTo(cv::Scalar(colour(rng), colour(rng), colour(rng)));
    // Hard-edged shapes give ORB plenty of corners; smooth gradients do not.
    for (int i = 0; i < 14; ++i) {
        const cv::Scalar tint(colour(rng), colour(rng), colour(rng));
        if (i % 3 == 0) {
            cv::circle(texture, {coordinate(rng), coordinate(rng)}, radius(rng), tint, cv::FILLED);
        } else if (i % 3 == 1) {
            cv::rectangle(texture, cv::Rect(coordinate(rng), coordinate(rng), radius(rng),
                                             radius(rng)),
                           tint, cv::FILLED);
        } else {
            cv::line(texture, {coordinate(rng), coordinate(rng)},
                      {coordinate(rng), coordinate(rng)}, tint, 3);
        }
    }
    return texture;
}

cv::Point3d operator+(const cv::Point3d& a, const cv::Point3d& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
cv::Point3d operator-(const cv::Point3d& a, const cv::Point3d& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
cv::Point3d operator*(const cv::Point3d& a, double s) { return {a.x * s, a.y * s, a.z * s}; }

std::vector<Patch> buildScene(std::mt19937& rng, double length) {
    std::vector<Patch> patches;
    std::uniform_real_distribution<double> along(0.5, length);
    std::uniform_real_distribution<double> height(-1.0, 1.2);
    std::uniform_real_distribution<double> depth(1.4, 3.2);
    std::uniform_real_distribution<double> tilt(-0.5, 0.5);
    std::uniform_real_distribution<double> size(0.18, 0.42);

    // Left wall, right wall, and a scattering of patches straight ahead, so
    // the camera always has structure at several distances.
    for (int i = 0; i < 150; ++i) {
        const bool left = (i % 2) == 0;
        Patch patch;
        const double side = left ? -depth(rng) : depth(rng);
        patch.centre = {side, height(rng), along(rng)};
        const double extent = size(rng);
        // Face roughly inwards, with a random tilt.
        patch.right = cv::Point3d(0.0, 0.0, 1.0) * extent + cv::Point3d(tilt(rng), 0, 0) * extent;
        patch.up = cv::Point3d(0.0, 1.0, 0.0) * extent + cv::Point3d(0, 0, tilt(rng)) * extent;
        patch.texture = makeTexture(rng, 96);
        patches.push_back(patch);
    }
    for (int i = 0; i < 40; ++i) {
        Patch patch;
        std::uniform_real_distribution<double> across(-1.8, 1.8);
        patch.centre = {across(rng), height(rng), along(rng) + 1.0};
        const double extent = size(rng);
        patch.right = cv::Point3d(1.0, 0.0, 0.0) * extent + cv::Point3d(0, 0, tilt(rng)) * extent;
        patch.up = cv::Point3d(0.0, 1.0, 0.0) * extent + cv::Point3d(0, 0, tilt(rng)) * extent;
        patch.texture = makeTexture(rng, 96);
        patches.push_back(patch);
    }
    return patches;
}

}  // namespace

int main(int argc, char** argv) {
    const common::CommandLine args(argc, argv);
    if (args.has("help")) {
        std::cout << "Render a synthetic corridor flythrough with ground truth.\n\n"
                     "  --out <file.avi>       video to write   [tests/data/synthetic.avi]\n"
                     "  --truth <file.txt>     ground-truth trajectory, TUM format\n"
                     "  --camera <file.yaml>   matching camera config to write\n"
                     "  --frames <n>           frame count      [420]\n"
                     "  --seed <n>             RNG seed         [7]\n";
        return 0;
    }

    const std::string videoPath = args.get("out", "tests/data/synthetic.avi");
    const std::string truthPath = args.get("truth", "tests/data/synthetic_truth.txt");
    const std::string cameraPath = args.get("camera", "tests/data/synthetic_camera.yaml");
    const int frameCount = args.getInt("frames", 420);

    const int width = 960;
    const int height = 720;
    const double fx = 900.0, fy = 900.0;
    const double cx = width / 2.0, cy = height / 2.0;
    const double fps = 30.0;

    std::mt19937 rng(static_cast<unsigned>(args.getInt("seed", 7)));
    const double corridorLength = 9.0;
    const std::vector<Patch> patches = buildScene(rng, corridorLength);

    cv::VideoWriter writer(videoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
                            cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "error: cannot open " << videoPath << " for writing\n";
        return 1;
    }

    std::ofstream truth(truthPath);
    if (!truth) {
        std::cerr << "error: cannot write " << truthPath << "\n";
        return 1;
    }
    truth << "# synthetic ground truth, TUM format: timestamp tx ty tz qx qy qz qw\n"
          << "# world frame is metres; camera frame is x right, y down, z forward\n";
    truth << std::fixed << std::setprecision(6);

    std::cout << "Rendering " << frameCount << " frames to " << videoPath << "\n";

    for (int index = 0; index < frameCount; ++index) {
        const double t = static_cast<double>(index) / static_cast<double>(frameCount);

        // A gentle S-curve down the corridor: sideways travel is what gives
        // monocular SLAM the baseline it needs to triangulate, so a pure
        // straight-ahead dolly would be a poor test.
        const cv::Point3d position{0.9 * std::sin(t * 2.0 * M_PI), 0.25 * std::sin(t * 4.0 * M_PI),
                                    0.6 + t * corridorLength * 0.75};
        const double yaw = 0.35 * std::sin(t * 2.0 * M_PI);

        // Camera looks along +z with a yaw wobble; camera axes are x right,
        // y down, z forward, matching ORB-SLAM3's convention.
        cv::Matx33d Rwc(std::cos(yaw), 0.0, std::sin(yaw),
                         0.0, 1.0, 0.0,
                         -std::sin(yaw), 0.0, std::cos(yaw));
        const cv::Matx33d Rcw = Rwc.t();
        const cv::Point3d twc = position;

        cv::Mat frame(height, width, CV_8UC3, cv::Scalar(28, 26, 24));
        cv::Mat depthBuffer(height, width, CV_32F, cv::Scalar(1e9f));

        // Painter's algorithm: draw far patches first so near ones overwrite.
        std::vector<std::pair<double, const Patch*>> ordered;
        ordered.reserve(patches.size());
        for (const auto& patch : patches) {
            const cv::Point3d relative = patch.centre - twc;
            const cv::Vec3d camera = Rcw * cv::Vec3d(relative.x, relative.y, relative.z);
            if (camera[2] > 0.35) ordered.emplace_back(camera[2], &patch);
        }
        std::sort(ordered.begin(), ordered.end(),
                   [](const auto& a, const auto& b) { return a.first > b.first; });

        for (const auto& [distance, patch] : ordered) {
            const cv::Point3d corners3d[4] = {
                patch->centre - patch->right + patch->up,
                patch->centre + patch->right + patch->up,
                patch->centre + patch->right - patch->up,
                patch->centre - patch->right - patch->up,
            };

            cv::Point2f imageCorners[4];
            bool visible = true;
            for (int i = 0; i < 4; ++i) {
                const cv::Point3d relative = corners3d[i] - twc;
                const cv::Vec3d camera = Rcw * cv::Vec3d(relative.x, relative.y, relative.z);
                if (camera[2] < 0.25) {
                    visible = false;
                    break;
                }
                imageCorners[i] = cv::Point2f(
                    static_cast<float>(fx * camera[0] / camera[2] + cx),
                    static_cast<float>(fy * camera[1] / camera[2] + cy));
            }
            if (!visible) continue;

            const int size = patch->texture.cols;
            const cv::Point2f textureCorners[4] = {{0.0f, 0.0f},
                                                    {static_cast<float>(size - 1), 0.0f},
                                                    {static_cast<float>(size - 1),
                                                     static_cast<float>(size - 1)},
                                                    {0.0f, static_cast<float>(size - 1)}};

            const cv::Mat homography =
                cv::getPerspectiveTransform(textureCorners, imageCorners);

            cv::Mat warped;
            cv::warpPerspective(patch->texture, warped, homography, frame.size(), cv::INTER_LINEAR,
                                 cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            cv::Mat mask(frame.size(), CV_8U, cv::Scalar(0));
            std::vector<cv::Point> polygon;
            for (const auto& corner : imageCorners) {
                polygon.emplace_back(cv::Point(cvRound(corner.x), cvRound(corner.y)));
            }
            cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
            warped.copyTo(frame, mask);
        }

        writer.write(frame);

        // TUM ground truth is world-from-camera: position, then orientation
        // as a quaternion.
        const cv::Matx33d R = Rwc;
        const double trace = R(0, 0) + R(1, 1) + R(2, 2);
        double qw, qx, qy, qz;
        if (trace > 0.0) {
            const double s = std::sqrt(trace + 1.0) * 2.0;
            qw = 0.25 * s;
            qx = (R(2, 1) - R(1, 2)) / s;
            qy = (R(0, 2) - R(2, 0)) / s;
            qz = (R(1, 0) - R(0, 1)) / s;
        } else {
            const double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
            qw = (R(2, 1) - R(1, 2)) / s;
            qx = 0.25 * s;
            qy = (R(0, 1) + R(1, 0)) / s;
            qz = (R(0, 2) + R(2, 0)) / s;
        }
        truth << (static_cast<double>(index) / fps) << " " << position.x << " " << position.y
              << " " << position.z << " " << qx << " " << qy << " " << qz << " " << qw << "\n";

        if (index % 60 == 0) std::cout << "  " << index << "/" << frameCount << "\n";
    }

    writer.release();
    truth.close();

    std::ofstream cameraConfig(cameraPath);
    // std::fixed matters: OpenCV's FileStorage types "900" as an integer
    // node, and ORB-SLAM3 aborts on a camera parameter that is not a real
    // number. Every float in this file must carry a decimal point.
    cameraConfig << std::fixed << std::setprecision(6);
    cameraConfig << "%YAML:1.0\n\n"
                 << "# Exact intrinsics of the synthetic renderer - no distortion, because\n"
                 << "# the renderer applies none. Used by the end-to-end pipeline test.\n\n"
                 << "File.version: \"1.0\"\n\nCamera.type: \"PinHole\"\n\n"
                 << "Camera1.fx: " << fx << "\nCamera1.fy: " << fy << "\n"
                 << "Camera1.cx: " << cx << "\nCamera1.cy: " << cy << "\n\n"
                 << "Camera1.k1: 0.0\nCamera1.k2: 0.0\nCamera1.p1: 0.0\nCamera1.p2: 0.0\n"
                 << "Camera1.k3: 0.0\n\n"
                 << "Camera.width: " << width << "\nCamera.height: " << height << "\n"
                 << "Camera.fps: " << static_cast<int>(fps) << "\nCamera.RGB: 0\n\n"
                 << "ORBextractor.nFeatures: 1500\nORBextractor.scaleFactor: 1.2\n"
                 << "ORBextractor.nLevels: 8\nORBextractor.iniThFAST: 20\n"
                 << "ORBextractor.minThFAST: 7\n\n"
                 << "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\n"
                 << "Viewer.GraphLineWidth: 0.9\nViewer.PointSize: 2.0\n"
                 << "Viewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\n"
                 << "Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\n"
                 << "Viewer.ViewpointZ: -3.5\nViewer.ViewpointF: 500.0\n";

    std::cout << "\nVideo:      " << videoPath << "\nGround truth: " << truthPath
              << "\nCamera:     " << cameraPath << "\n";
    return 0;
}
