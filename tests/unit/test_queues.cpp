#include <optional>
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
    testBoundedQueueDropsOldestWhenFull();
    return test::summary("queues");
}
