#pragma once

#include <charconv>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace common {

// Minimal `--key value` / `--flag` parser, shared by every app so their
// option handling stays consistent and none of them grows its own argv loop.
class CommandLine {
public:
    CommandLine(int argc, char** argv) {
        if (argc > 0) program_ = argv[0];
        for (int i = 1; i < argc; ++i) args_.emplace_back(argv[i]);
    }

    bool has(std::string_view name) const {
        for (const auto& arg : args_) {
            if (arg == std::string("--") + std::string(name)) return true;
        }
        return false;
    }

    std::string get(std::string_view name, std::string fallback = {}) const {
        const std::string key = std::string("--") + std::string(name);
        for (std::size_t i = 0; i + 1 < args_.size(); ++i) {
            if (args_[i] == key) return args_[i + 1];
        }
        // Also accept --key=value.
        for (const auto& arg : args_) {
            if (arg.rfind(key + "=", 0) == 0) return arg.substr(key.size() + 1);
        }
        return fallback;
    }

    int getInt(std::string_view name, int fallback) const {
        const std::string value = get(name);
        if (value.empty()) return fallback;
        int parsed = fallback;
        const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
        return result.ec == std::errc{} ? parsed : fallback;
    }

    double getDouble(std::string_view name, double fallback) const {
        const std::string value = get(name);
        if (value.empty()) return fallback;
        try {
            return std::stod(value);
        } catch (...) {
            return fallback;
        }
    }

    const std::string& program() const { return program_; }

private:
    std::string program_;
    std::vector<std::string> args_;
};

} // namespace common
