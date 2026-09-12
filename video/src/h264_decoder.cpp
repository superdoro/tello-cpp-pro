#include "video/h264_decoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/log.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cstdlib>

#include <opencv2/imgproc.hpp>

#include "common/logging.hpp"

namespace video {

struct H264Decoder::Impl {
    const AVCodec* codec = nullptr;
    AVCodecContext* context = nullptr;
    AVCodecParserContext* parser = nullptr;
    AVPacket* packet = nullptr;
    AVFrame* frame = nullptr;
    SwsContext* scaler = nullptr;
    int scaler_width = 0;
    int scaler_height = 0;
    int scaler_format = -1;
    std::uint64_t error_count = 0;

    ~Impl() { teardown(); }

    void teardown() {
        if (scaler) { sws_freeContext(scaler); scaler = nullptr; }
        if (frame) { av_frame_free(&frame); }
        if (packet) { av_packet_free(&packet); }
        if (parser) { av_parser_close(parser); parser = nullptr; }
        if (context) { avcodec_free_context(&context); }
    }

    // Converts the decoder's native (usually YUV420P) output to BGR8.
    bool toBgr(cv::Mat& out) {
        if (frame->width <= 0 || frame->height <= 0) return false;

        if (!scaler || scaler_width != frame->width || scaler_height != frame->height ||
            scaler_format != frame->format) {
            if (scaler) sws_freeContext(scaler);
            scaler = sws_getContext(frame->width, frame->height,
                                     static_cast<AVPixelFormat>(frame->format),
                                     frame->width, frame->height, AV_PIX_FMT_BGR24,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (!scaler) return false;
            scaler_width = frame->width;
            scaler_height = frame->height;
            scaler_format = frame->format;
        }

        out.create(frame->height, frame->width, CV_8UC3);
        std::uint8_t* dst[4] = {out.data, nullptr, nullptr, nullptr};
        int dstStride[4] = {static_cast<int>(out.step[0]), 0, 0, 0};
        sws_scale(scaler, frame->data, frame->linesize, 0, frame->height, dst, dstStride);
        return true;
    }
};

H264Decoder::H264Decoder() : impl_(std::make_unique<Impl>()) {}
H264Decoder::~H264Decoder() = default;

bool H264Decoder::open() {
    close();

    // libavcodec logs every concealed macroblock to stderr. On a lossy UDP
    // link that is a continuous flood that buries this program's own output -
    // including the messages explaining what is wrong. We already count
    // errors ourselves and surface them through errorCount(), so the raw
    // stream of decoder complaints is silenced unless explicitly asked for.
    av_log_set_level(std::getenv("TELLO_FFMPEG_LOG") ? AV_LOG_WARNING : AV_LOG_QUIET);

    impl_->codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!impl_->codec) {
        common::logError("H264Decoder", "no H264 decoder available in this libavcodec build");
        return false;
    }

    impl_->parser = av_parser_init(impl_->codec->id);
    impl_->context = avcodec_alloc_context3(impl_->codec);
    impl_->packet = av_packet_alloc();
    impl_->frame = av_frame_alloc();
    if (!impl_->parser || !impl_->context || !impl_->packet || !impl_->frame) {
        common::logError("H264Decoder", "failed to allocate decoder resources");
        close();
        return false;
    }

    // Latency, not throughput, is what matters for closed-loop flight: a
    // frame that arrives 200ms late is worse than useless as control
    // feedback. LOW_DELAY tells the decoder not to buffer frames for
    // reordering, and error concealment keeps partially-received frames
    // instead of dropping them, which matters on a lossy UDP link.
    impl_->context->flags |= AV_CODEC_FLAG_LOW_DELAY;
    impl_->context->flags2 |= AV_CODEC_FLAG2_FAST;
    impl_->context->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
    impl_->context->thread_type = FF_THREAD_SLICE;

    // AV_CODEC_FLAG2_CHUNKS is deliberately NOT set, though it looks like
    // exactly the right flag for a stream arriving in UDP datagrams.
    //
    // It tells the decoder that a packet may hold only part of a frame, which
    // is a promise the parser above already keeps - av_parser_parse2 hands
    // over whole access units. Setting it as well makes the decoder defer
    // finalising each frame until the next one starts, and on a lossy link
    // that wedges it permanently: measured against a real Tello recording,
    // the decoder produced 538 of 843 frames and then emitted nothing for the
    // remaining 4 MB of a healthy stream. With ~5% datagram loss injected it
    // produced zero frames at all. Without the flag: 838 of 843.

    if (avcodec_open2(impl_->context, impl_->codec, nullptr) < 0) {
        common::logError("H264Decoder", "avcodec_open2 failed");
        close();
        return false;
    }
    return true;
}

void H264Decoder::close() { impl_->teardown(); }

bool H264Decoder::isOpen() const { return impl_->context != nullptr; }

std::uint64_t H264Decoder::errorCount() const { return impl_->error_count; }

bool H264Decoder::decode(const std::uint8_t* data, std::size_t size, std::vector<cv::Mat>& out) {
    if (!isOpen()) return false;

    while (size > 0) {
        std::uint8_t* parsedData = nullptr;
        int parsedSize = 0;
        const int consumed = av_parser_parse2(
            impl_->parser, impl_->context, &parsedData, &parsedSize, data,
            static_cast<int>(size), AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        if (consumed < 0) {
            ++impl_->error_count;
            return true;  // resync on the next datagram rather than tearing down
        }
        data += consumed;
        size -= static_cast<std::size_t>(consumed);

        if (parsedSize == 0) continue;

        impl_->packet->data = parsedData;
        impl_->packet->size = parsedSize;

        if (avcodec_send_packet(impl_->context, impl_->packet) < 0) {
            ++impl_->error_count;
            continue;
        }

        while (true) {
            const int ret = avcodec_receive_frame(impl_->context, impl_->frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
            if (ret < 0) {
                ++impl_->error_count;
                break;
            }
            cv::Mat bgr;
            if (impl_->toBgr(bgr)) {
                out.push_back(std::move(bgr));
            }
            av_frame_unref(impl_->frame);
        }
    }
    return true;
}

} // namespace video
