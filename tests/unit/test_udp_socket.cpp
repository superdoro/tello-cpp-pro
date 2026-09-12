// The socket's timeout behaviour, which is easy to get subtly and
// catastrophically wrong.
//
// drain() once hung the whole program on startup: it asked for a zero
// timeout, receiveInto() turned that into a zero timeval for SO_RCVTIMEO,
// and POSIX reads a zero timeval as "block forever" - the exact opposite of
// what was meant. The drone connect path stopped dead with no output at all.
#include <chrono>
#include <thread>

#include "common/net/udp_socket.hpp"
#include "test_check.hpp"

namespace {

using Clock = std::chrono::steady_clock;

int elapsedMs(Clock::time_point start) {
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count());
}

void testDrainReturnsImmediatelyOnAnEmptySocket() {
    test::beginCase("drain() on an idle socket returns at once, never blocks");
    common::net::UdpSocket socket;
    CHECK(socket.bind(45871));

    const auto start = Clock::now();
    const std::size_t discarded = socket.drain();
    const int took = elapsedMs(start);

    CHECK_EQ(discarded, 0u);
    CHECK(took < 200);
}

void testDrainDiscardsPendingDatagrams() {
    test::beginCase("drain() discards what is already queued");
    common::net::UdpSocket receiver;
    CHECK(receiver.bind(45872));

    common::net::UdpSocket sender;
    CHECK(sender.bind(0));
    CHECK(sender.sendTo("127.0.0.1", 45872, "stale one"));
    CHECK(sender.sendTo("127.0.0.1", 45872, "stale two"));

    // Give the loopback a moment to deliver.
    for (int i = 0; i < 50 && receiver.drain() == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Whatever was queued is gone, and the next read finds nothing.
    const auto start = Clock::now();
    CHECK(!receiver.receive(0).has_value());
    CHECK(elapsedMs(start) < 200);
}

void testReceiveHonoursItsTimeout() {
    test::beginCase("receive() waits about as long as asked, then gives up");
    common::net::UdpSocket socket;
    CHECK(socket.bind(45873));

    const auto start = Clock::now();
    const auto result = socket.receive(300);
    const int took = elapsedMs(start);

    CHECK(!result.has_value());
    CHECK(took >= 250);
    CHECK(took < 1500);
}

}  // namespace

int main() {
    std::cout << "udp_socket\n";
    testDrainReturnsImmediatelyOnAnEmptySocket();
    testDrainDiscardsPendingDatagrams();
    testReceiveHonoursItsTimeout();
    return test::summary("udp_socket");
}
