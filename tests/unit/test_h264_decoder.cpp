// Feeds a raw Annex-B H264 stream through the decoder in ~1460-byte pieces,
// the way the Tello's UDP datagrams arrive, and with datagrams missing.
//
// This is the exact path that failed on real hardware: video ran for about
// 18 seconds, then stopped dead while data kept pouring in. The cause was a
// decoder flag (AV_CODEC_FLAG2_CHUNKS) that wedges permanently once a frame
// arrives incomplete. Nothing caught it, because a short, clean, low-bitrate
// synthetic clip decodes identically with the flag set or clear.
//
// So the fixture here is built to have the two properties that matter:
//   * frames much larger than one datagram, so NALs span many pieces;
//   * simulated packet loss, so frames arrive incomplete.
// Without both, this test passes against a decoder that cannot survive a
// real flight.
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "test_check.hpp"
#include "video/h264_decoder.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr int kFramesPerPass = 45;
constexpr int kPasses = 3;
constexpr std::size_t kDatagram = 1460;  // the Tello's payload size

bool writeRawStream(const std::string& path) {
    cv::VideoWriter writer(path, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30,
                            cv::Size(kWidth, kHeight));
    if (!writer.isOpened()) return false;

    for (int i = 0; i < kFramesPerPass; ++i) {
        // Noise is incompressible on purpose: it forces frames far bigger
        // than a datagram, which is what makes the chunking behaviour matter.
        cv::Mat frame(kHeight, kWidth, CV_8UC3);
        cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        cv::rectangle(frame, cv::Rect((i * 5) % (kWidth - 40), 40, 40, 40),
                       cv::Scalar(255, 255, 255), cv::FILLED);
        writer.write(frame);
    }
    writer.release();
    return true;
}

std::vector<std::uint8_t> readFile(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(input)),
                                      std::istreambuf_iterator<char>());
}

struct Replay {
    std::size_t frames = 0;
    std::size_t chunks = 0;
    std::size_t lastChunkWithFrame = 0;
};

// dropEvery == 0 means a clean link; otherwise one datagram in that many is
// thrown away.
Replay replay(const std::vector<std::uint8_t>& stream, std::size_t from, int dropEvery) {
    video::H264Decoder decoder;
    if (!decoder.open()) return {};

    Replay result;
    std::vector<cv::Mat> frames;
    for (std::size_t offset = from; offset < stream.size(); offset += kDatagram, ++result.chunks) {
        if (dropEvery > 0 && result.chunks % static_cast<std::size_t>(dropEvery) == 3) continue;

        const std::size_t size = std::min(kDatagram, stream.size() - offset);
        frames.clear();
        if (!decoder.decode(stream.data() + offset, size, frames)) break;
        if (!frames.empty()) {
            result.frames += frames.size();
            result.lastChunkWithFrame = result.chunks;
        }
    }
    return result;
}

void testDecodesStreamSplitAcrossDatagrams(const std::vector<std::uint8_t>& stream) {
    test::beginCase("a stream chopped into datagram-sized pieces decodes end to end");
    const Replay result = replay(stream, 0, 0);

    CHECK(result.frames > static_cast<std::size_t>(kFramesPerPass * kPasses) / 2);
    // Frames must still be arriving at the end, not just at the start.
    CHECK(result.lastChunkWithFrame > result.chunks * 3 / 4);
}

void testSurvivesLostDatagrams(const std::vector<std::uint8_t>& stream) {
    test::beginCase("packet loss costs frames but never wedges the decoder");
    const Replay clean = replay(stream, 0, 0);
    const Replay lossy = replay(stream, 0, 20);  // ~5% of datagrams dropped

    // The regression guard. With AV_CODEC_FLAG2_CHUNKS set, this case decoded
    // exactly zero frames while the clean case decoded every one - the decoder
    // never recovers from its first incomplete frame.
    CHECK(lossy.frames > clean.frames / 2);
    CHECK(lossy.lastChunkWithFrame > lossy.chunks * 3 / 4);
}

void testMidStreamStart(const std::vector<std::uint8_t>& stream) {
    test::beginCase("joining a stream already in progress recovers at the next keyframe");
    // A drone is already broadcasting when we bind the socket, so the first
    // bytes we ever see are mid-frame with no leading SPS/PPS. Recovery has
    // to wait for the next keyframe, and must then actually happen.
    const Replay result = replay(stream, stream.size() / 3, 0);
    CHECK(result.frames > 0);
    CHECK(result.lastChunkWithFrame > result.chunks * 3 / 4);
}

}  // namespace

int main() {
    std::cout << "h264_decoder\n";

    const std::string path = "test_h264_decoder_fixture.h264";
    if (!writeRawStream(path)) {
        std::cout << "  SKIP: this OpenCV build cannot write raw H264\n";
        return 0;
    }
    const std::vector<std::uint8_t> base = readFile(path);
    std::remove(path.c_str());

    if (base.empty()) {
        std::cout << "  SKIP: fixture stream is empty\n";
        return 0;
    }

    // The encoder emits SPS/PPS and an IDR exactly once, at the start, then
    // nothing but P frames - a file, not a broadcast. A live stream repeats
    // them periodically or no client could ever tune in. Concatenating the
    // encoded stream models that: each copy begins with a fresh SPS/PPS/IDR.
    std::vector<std::uint8_t> stream;
    for (int pass = 0; pass < kPasses; ++pass) {
        stream.insert(stream.end(), base.begin(), base.end());
    }

    testDecodesStreamSplitAcrossDatagrams(stream);
    testSurvivesLostDatagrams(stream);
    testMidStreamStart(stream);
    return test::summary("h264_decoder");
}
