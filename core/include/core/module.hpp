#pragma once

#include <stop_token>
#include <string>
#include <thread>

namespace core {

// Lifecycle contract every long-running module (drivers, slam, control, ...)
// implements. apps/ wires modules together and drives start()/stop(); no
// module reaches into another module's internals directly, only through the
// EventBus.
class IModule {
public:
    virtual ~IModule() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual const std::string& name() const = 0;
};

// Convenience base for a module that owns a single worker thread running a
// loop until stop() is requested.
class ThreadedModule : public IModule {
public:
    explicit ThreadedModule(std::string name) : name_(std::move(name)) {}
    ~ThreadedModule() override { ThreadedModule::stop(); }

    void start() override {
        thread_ = std::jthread([this](std::stop_token token) { run(token); });
    }

    void stop() override {
        if (thread_.joinable()) {
            thread_.request_stop();
            thread_.join();
        }
    }

    const std::string& name() const override { return name_; }

protected:
    virtual void run(std::stop_token stopToken) = 0;

private:
    std::string name_;
    std::jthread thread_;
};

} // namespace core
