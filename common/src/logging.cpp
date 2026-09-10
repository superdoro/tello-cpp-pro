#include "common/logging.hpp"

#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>

namespace common {

namespace {

std::atomic<LogLevel> g_minLevel{LogLevel::Info};
std::mutex g_outputMutex;

const char* levelName(LogLevel level) {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warn: return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

void writeLine(LogLevel level, std::string_view component, std::string_view message) {
    if (level < g_minLevel.load(std::memory_order_relaxed)) return;

    const auto now = std::chrono::system_clock::now();
    const auto nowTimeT = std::chrono::system_clock::to_time_t(now);
    std::tm tmBuf{};
    localtime_r(&nowTimeT, &tmBuf);

    std::lock_guard lock(g_outputMutex);
    auto& out = (level == LogLevel::Error || level == LogLevel::Warn) ? std::cerr : std::cout;
    out << std::put_time(&tmBuf, "%H:%M:%S") << " [" << levelName(level) << "] " << component
        << ": " << message << '\n';
}

} // namespace

void setMinLogLevel(LogLevel level) { g_minLevel.store(level, std::memory_order_relaxed); }

void logDebug(std::string_view component, std::string_view message) {
    writeLine(LogLevel::Debug, component, message);
}
void logInfo(std::string_view component, std::string_view message) {
    writeLine(LogLevel::Info, component, message);
}
void logWarn(std::string_view component, std::string_view message) {
    writeLine(LogLevel::Warn, component, message);
}
void logError(std::string_view component, std::string_view message) {
    writeLine(LogLevel::Error, component, message);
}

} // namespace common
