#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace common::net {

// Thin RAII wrapper around a POSIX UDP socket, shared by every module that
// speaks to the drone: the command/state channels in drivers/tello and the
// raw H264 video stream in video/.
class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Binds to receive datagrams on the given local port (0.0.0.0). Returns
    // false on failure.
    bool bind(std::uint16_t localPort);

    // Grows the kernel receive buffer. The video stream bursts a whole frame
    // as a rapid train of ~1460-byte datagrams, so the default buffer can
    // overflow between reads and silently shred frames.
    bool setReceiveBufferBytes(int bytes);

    bool sendTo(const std::string& host, std::uint16_t port, const std::string& data);

    // Blocks up to timeoutMs waiting for one datagram; returns nullopt on
    // timeout or error.
    std::optional<std::string> receive(int timeoutMs);

    // Zero-copy variant for the video path: writes one datagram into the
    // caller's buffer and returns its length, or 0 on timeout/error.
    std::size_t receiveInto(void* buffer, std::size_t capacity, int timeoutMs);

    // Discards whatever is already queued, without blocking. Needed before
    // a request/response exchange on a socket that has no way to correlate a
    // reply with its request: a late ack from an earlier command would
    // otherwise be read as the answer to this one.
    std::size_t drain();

    bool isOpen() const { return fd_ >= 0; }

    void close();

private:
    int fd_ = -1;
};

} // namespace common::net
