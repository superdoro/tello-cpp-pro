#include <chrono>
#include <optional>
#include <stop_token>
#include <thread>

#include "common/queue/bounded_overwrite_queue.hpp"
#include "common/queue/latest_value_box.hpp"
#include "test_check.hpp"

namespace {

void testLatestValueBoxKeepsOnlyTheNewest() {
    test::beginCase("the latest-value box discards stale values");
    common::queue::LatestValueBox<int> box;
    CHECK(!box.get().has_value());

    box.set(1);
    box.set(2);
    box.set(3);
    CHECK_EQ(box.get().value(), 3);
}

void testTakeEmptiesTheBox() {
    test::beginCase("take() removes the value, get() does not");
    common::queue::LatestValueBox<int> box;
    box.set(7);

    CHECK_EQ(box.get().value(), 7);
    CHECK_EQ(box.get().value(), 7);  // get is non-destructive
    CHECK(!box.empty());

    CHECK_EQ(box.take().value(), 7);
    CHECK(box.empty());
    CHECK(!box.take().has_value());
}

void testWaitAndTakeWakesOnANewValue() {
    test::beginCase("waitAndTake returns a value published by another thread");
    common::queue::LatestValueBox<int> box;
    std::stop_source stop;

    std::thread producer([&box] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        box.set(42);
    });

    const auto value = box.waitAndTake(stop.get_token(), std::chrono::milliseconds(2000));
    producer.join();

    CHECK(value.has_value());
    CHECK_EQ(value.value(), 42);
    CHECK(box.empty());
}

void testWaitAndTakeTimesOutEmpty() {
    test::beginCase("waitAndTake returns nullopt when nothing arrives");
    common::queue::LatestValueBox<int> box;
    std::stop_source stop;

    const auto startedAt = std::chrono::steady_clock::now();
    const auto value = box.waitAndTake(stop.get_token(), std::chrono::milliseconds(30));
    const auto elapsed = std::chrono::steady_clock::now() - startedAt;

    CHECK(!value.has_value());
    // It must actually have waited, not spun through.
    CHECK(elapsed >= std::chrono::milliseconds(25));
}

void testWaitAndTakeReturnsPromptlyOnStop() {
    test::beginCase("waitAndTake returns as soon as stop is requested");
    // This is the property a ThreadedModule run loop depends on: without it,
    // stop() blocks for the full timeout on every shutdown.
    common::queue::LatestValueBox<int> box;
    std::stop_source stop;

    std::thread stopper([&stop] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        stop.request_stop();
    });

    const auto startedAt = std::chrono::steady_clock::now();
    const auto value = box.waitAndTake(stop.get_token(), std::chrono::seconds(10));
    const auto elapsed = std::chrono::steady_clock::now() - startedAt;
    stopper.join();

    CHECK(!value.has_value());
    CHECK(elapsed < std::chrono::seconds(2));
}

void testWaitAndTakeReturnsImmediatelyIfAlreadyStopped() {
    test::beginCase("waitAndTake does not block when stop was already requested");
    common::queue::LatestValueBox<int> box;
    std::stop_source stop;
    stop.request_stop();

    const auto startedAt = std::chrono::steady_clock::now();
    const auto value = box.waitAndTake(stop.get_token(), std::chrono::seconds(10));
    const auto elapsed = std::chrono::steady_clock::now() - startedAt;

    CHECK(!value.has_value());
    CHECK(elapsed < std::chrono::seconds(1));
}

void testBoundedQueueDropsOldestWhenFull() {
    test::beginCase("the bounded queue drops the oldest entry, not the newest");
    common::queue::BoundedOverwriteQueue<int> queue(2);
    queue.push(1);
    queue.push(2);
    queue.push(3);  // evicts 1

    CHECK_EQ(queue.tryPop().value(), 2);
    CHECK_EQ(queue.tryPop().value(), 3);
    CHECK(!queue.tryPop().has_value());
}

}  // namespace

int main() {
    std::cout << "queues\n";
    testLatestValueBoxKeepsOnlyTheNewest();
    testTakeEmptiesTheBox();
    testWaitAndTakeWakesOnANewValue();
    testWaitAndTakeTimesOutEmpty();
    testWaitAndTakeReturnsPromptlyOnStop();
    testWaitAndTakeReturnsImmediatelyIfAlreadyStopped();
    testBoundedQueueDropsOldestWhenFull();
    return test::summary("queues");
}
