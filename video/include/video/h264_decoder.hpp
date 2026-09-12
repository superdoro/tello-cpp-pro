#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace video {

// Decodes a raw H264 elementary stream (no container) into BGR images.
//
// The Tello broadcasts exactly that: bare Annex-B NAL units chopped across
// ~1460-byte UDP datagrams, with no RTP framing and no reliability. Rather
// than trying to reassemble frames ourselves, we push every received byte
// into libavcodec's H264 parser, which is already built to find NAL start
// codes in an arbitrarily-chunked byte stream and to resynchronise after
// packet loss.
class H264Decoder {
public:
    H264Decoder();
    ~H264Decoder();

    H264Decoder(const H264Decoder&) = delete;
    H264Decoder& operator=(const H264Decoder&) = delete;

    bool open();
    void close();
    bool isOpen() const;

    // Feeds `size` bytes of stream and appends every frame that completed as
    // a result to `out` (usually 0 or 1, occasionally more). Returns false
    // only on an unrecoverable decoder error; corrupt/lost input is normal
    // and simply yields no frames.
    bool decode(const std::uint8_t* data, std::size_t size, std::vector<cv::Mat>& out);

    // Number of frames libavcodec reported as corrupt/dropped, for diagnostics.
    std::uint64_t errorCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace video
