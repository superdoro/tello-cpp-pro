#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace drivers::tello::detail {

// Thin RAII wrapper around a POSIX UDP socket. Implementation detail of
// TelloDriver, not part of the module's public interface.
class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Binds to receive datagrams on the given local port (0.0.0.0). Returns
    // false on failure.
    bool bind(std::uint16_t localPort);

    bool sendTo(const std::string& host, std::uint16_t port, const std::string& data);

    // Blocks up to timeoutMs waiting for one datagram; returns nullopt on
    // timeout or error.
    std::optional<std::string> receive(int timeoutMs);

    void close();

private:
    int fd_ = -1;
};

} // namespace drivers::tello::detail
