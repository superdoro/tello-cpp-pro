#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

namespace common::queue {

// Bounded queue that drops the oldest entry when full, for streams (e.g.
// outgoing velocity commands) where every value should be seen at least once
// by a consumer but backlog must not grow unbounded.
template <typename T>
class BoundedOverwriteQueue {
public:
    explicit BoundedOverwriteQueue(std::size_t capacity) : capacity_(capacity) {}

    void push(T value) {
        std::lock_guard lock(mutex_);
        if (items_.size() >= capacity_) {
            items_.pop_front();
        }
        items_.push_back(std::move(value));
        cv_.notify_one();
    }

    std::optional<T> tryPop() {
        std::lock_guard lock(mutex_);
        if (items_.empty()) return std::nullopt;
        T value = std::move(items_.front());
        items_.pop_front();
        return value;
    }

private:
    std::size_t capacity_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> items_;
};

} // namespace common::queue
