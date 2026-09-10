#pragma once

#include <string_view>

// Minimal dependency-free logging facade. Every module includes only this
// header, never a concrete logging backend directly, so the implementation
// in logging.cpp can later be swapped for a library like spdlog without
// touching any call site.
namespace common {

enum class LogLevel { Debug, Info, Warn, Error };

void setMinLogLevel(LogLevel level);

void logDebug(std::string_view component, std::string_view message);
void logInfo(std::string_view component, std::string_view message);
void logWarn(std::string_view component, std::string_view message);
void logError(std::string_view component, std::string_view message);

} // namespace common
