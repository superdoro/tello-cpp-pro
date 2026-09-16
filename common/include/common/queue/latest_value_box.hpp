#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <stop_token>

namespace common::queue {

// Thread-safe single-slot holder for "latest wins" data such as video frames
// or pose estimates, where a consumer should always see the newest value and
// stale backlog must never accumulate.
//
// The overwrite is the point, not a limitation: a producer that outruns its
// consumer must not build a queue, because every queued item is a frame the
// consumer will act on after it has stopped being true. Dropping is the
// correct behaviour for perception and control data alike.
template <typename T>
class LatestValueBox {
public:
    void set(T value) {
        {
            std::lock_guard lock(mutex_);
            value_ = std::move(value);
        }
        // Outside the lock: a woken waiter would otherwise immediately block
        // again on a mutex we still hold.
        ready_.notify_one();
    }

    std::optional<T> get() const {
        std::lock_guard lock(mutex_);
        return value_;
    }

    // Removes and returns the value, leaving the box empty. A consumer that
    // polls with get() cannot tell a fresh value from the one it already
    // handled; this can.
    std::optional<T> take() {
        std::lock_guard lock(mutex_);
        std::optional<T> taken;
        taken.swap(value_);
        return taken;
    }

    // Blocks until a value is available, `timeout` elapses, or `stopToken` is
    // signalled, then takes it. Returns nullopt on timeout or stop.
    //
    // The stop_token overload is what makes this usable from a
    // core::ThreadedModule run loop: without it the only way to stay
    // responsive to stop() is to poll on a short timeout, which burns a wakeup
    // per interval forever.
    std::optional<T> waitAndTake(std::stop_token stopToken,
                                  std::chrono::milliseconds timeout) {
        std::unique_lock lock(mutex_);
        ready_.wait_for(lock, stopToken, timeout, [this] { return value_.has_value(); });
        std::optional<T> taken;
        taken.swap(value_);
        return taken;
    }

    bool empty() const {
        std::lock_guard lock(mutex_);
        return !value_.has_value();
    }

    void clear() {
        std::lock_guard lock(mutex_);
        value_.reset();
    }

private:
    mutable std::mutex mutex_;
    // condition_variable_any, not condition_variable: only the _any flavour
    // has the C++20 stop_token-aware wait overloads.
    std::condition_variable_any ready_;
    std::optional<T> value_;
};

} // namespace common::queue
