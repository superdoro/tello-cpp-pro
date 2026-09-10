#pragma once

#include <any>
#include <functional>
#include <mutex>
#include <optional>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace core {

// In-process, thread-safe, typed publish/subscribe bus. This is the only
// channel through which modules (drivers, slam, control, ...) exchange data,
// so that a future split into separate processes only requires swapping this
// class's implementation, not the modules that use it.
//
// Handlers are invoked synchronously on the publishing thread. Subscribers
// must therefore be fast/non-blocking (typically: copy the event into the
// subscriber's own queue and return), never do heavy work inline.
class EventBus {
public:
    template <typename T>
    using Handler = std::function<void(const T&)>;

    template <typename T>
    void subscribe(Handler<T> handler) {
        std::lock_guard lock(mutex_);
        handlers_[std::type_index(typeid(T))].push_back(
            [handler = std::move(handler)](const std::any& event) {
                handler(std::any_cast<const T&>(event));
            });
    }

    template <typename T>
    void publish(T event) {
        std::vector<AnyHandler> handlersCopy;
        {
            std::lock_guard lock(mutex_);
            if (auto it = handlers_.find(std::type_index(typeid(T))); it != handlers_.end()) {
                handlersCopy = it->second;
            }
            latest_[std::type_index(typeid(T))] = event;
        }
        std::any boxed = std::move(event);
        for (auto& handler : handlersCopy) {
            handler(boxed);
        }
    }

    // Polling-style access to the last published value of type T, for
    // consumers that don't want to subscribe (e.g. a CLI status printer).
    template <typename T>
    std::optional<T> latest() const {
        std::lock_guard lock(mutex_);
        auto it = latest_.find(std::type_index(typeid(T)));
        if (it == latest_.end()) return std::nullopt;
        return std::any_cast<T>(it->second);
    }

private:
    using AnyHandler = std::function<void(const std::any&)>;

    mutable std::mutex mutex_;
    std::unordered_map<std::type_index, std::vector<AnyHandler>> handlers_;
    std::unordered_map<std::type_index, std::any> latest_;
};

} // namespace core
