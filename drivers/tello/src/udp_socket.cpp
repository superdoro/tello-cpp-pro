#include "udp_socket.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace drivers::tello::detail {

UdpSocket::~UdpSocket() { close(); }

bool UdpSocket::bind(std::uint16_t localPort) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    int reuse = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(localPort);

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close();
        return false;
    }
    return true;
}

bool UdpSocket::sendTo(const std::string& host, std::uint16_t port, const std::string& data) {
    if (fd_ < 0) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) return false;

    const ssize_t sent = ::sendto(fd_, data.data(), data.size(), 0,
                                   reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    return sent == static_cast<ssize_t>(data.size());
}

std::optional<std::string> UdpSocket::receive(int timeoutMs) {
    if (fd_ < 0) return std::nullopt;

    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char buffer[2048];
    sockaddr_in fromAddr{};
    socklen_t fromLen = sizeof(fromAddr);
    const ssize_t received = ::recvfrom(fd_, buffer, sizeof(buffer) - 1, 0,
                                         reinterpret_cast<sockaddr*>(&fromAddr), &fromLen);
    if (received <= 0) return std::nullopt;

    return std::string(buffer, static_cast<std::size_t>(received));
}

void UdpSocket::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

} // namespace drivers::tello::detail
