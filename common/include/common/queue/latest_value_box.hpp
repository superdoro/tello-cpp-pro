#pragma once

#include <mutex>
#include <optional>

namespace common::queue {

// Thread-safe single-slot holder for "latest wins" data such as video frames
// or pose estimates, where a consumer should always see the newest value and
// stale backlog must never accumulate.
template <typename T>
class LatestValueBox {
public:
    void set(T value) {
        std::lock_guard lock(mutex_);
        value_ = std::move(value);
    }

    std::optional<T> get() const {
        std::lock_guard lock(mutex_);
        return value_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<T> value_;
};

} // namespace common::queue
