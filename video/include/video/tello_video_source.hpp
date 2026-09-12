#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "video/i_frame_source.hpp"

namespace video {

struct TelloVideoConfig {
    std::uint16_t port = 11111;

    // The drone bursts a whole frame as a train of ~1460-byte datagrams. The
    // default kernel buffer is easily overrun between reads, and a lost
    // datagram costs the whole frame (and, for an I-frame, everything until
    // the next one), so ask for a generous one.
    int receive_buffer_bytes = 2 * 1024 * 1024;

    // Optional: also write every received byte to this file, giving a raw
    // .h264 elementary stream that VideoFileSource can replay later. This is
    // what makes "record one flight, build the map offline" possible.
    std::string record_path;

    // See VideoFileConfig::target_width - the live stream must be delivered
    // at the same resolution the map was built at.
    int target_width = 0;
    int target_height = 0;
};

// Live frames from the Tello's H264 broadcast on UDP 11111.
//
// Owns a receive thread so a slow consumer (SLAM taking 40ms on a frame)
// never stalls the socket into dropping datagrams. Only the newest decoded
// frame is kept: for closed-loop control a backlog of stale frames is worse
// than a gap, since acting on a 500ms-old pose is how a drone flies into a
// wall.
//
// `streamon` is NOT sent from here - that is a command-channel concern owned
// by IDrone::enableVideoStream(). Call that first.
class TelloVideoSource : public IFrameSource {
public:
    explicit TelloVideoSource(TelloVideoConfig config);
    ~TelloVideoSource() override;

    bool open() override;
    void close() override;
    bool isOpen() const override;
    std::optional<Frame> nextFrame(int timeoutMs) override;
    double nominalFps() const override { return 30.0; }

    // Frames decoded, and frames decoded but never consumed because a newer
    // one arrived first. A large drop count means SLAM is not keeping up.
    std::uint64_t decodedCount() const;
    std::uint64_t droppedCount() const;

    // Raw link statistics, which exist to answer the one question that
    // matters when no video shows up: is the drone sending anything at all?
    // Zero datagrams is a network problem (firewall, wrong interface, a
    // streamon that did not take). Datagrams but no decoded frames is a
    // stream problem (waiting for the first keyframe, or heavy packet loss).
    std::uint64_t receivedDatagrams() const;
    std::uint64_t receivedBytes() const;
    std::uint64_t decoderErrors() const;

    std::string healthSummary() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace video
