#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

// A ~40-line test harness instead of a dependency. The project's testable
// surface is small and its value is in being runnable anywhere with nothing
// installed, which matters more here than the ergonomics a real framework
// would add.
namespace test {

inline int g_failures = 0;
inline std::string g_currentCase;

inline void beginCase(const std::string& name) {
    g_currentCase = name;
    std::cout << "  " << name << "\n";
}

inline void fail(const std::string& message, const char* file, int line) {
    ++g_failures;
    std::cout << "    FAIL " << file << ":" << line << "  " << message << "\n";
}

inline int summary(const std::string& suite) {
    if (g_failures == 0) {
        std::cout << suite << ": all checks passed\n";
        return 0;
    }
    std::cout << suite << ": " << g_failures << " check(s) FAILED\n";
    return 1;
}

}  // namespace test

#define CHECK(condition)                                                          \
    do {                                                                          \
        if (!(condition)) test::fail("expected: " #condition, __FILE__, __LINE__); \
    } while (false)

#define CHECK_NEAR(actual, expected, tolerance)                                             \
    do {                                                                                    \
        const double a_ = (actual);                                                         \
        const double e_ = (expected);                                                       \
        if (std::abs(a_ - e_) > (tolerance)) {                                              \
            test::fail(std::string(#actual) + " = " + std::to_string(a_) + ", expected " +  \
                            std::to_string(e_) + " +/- " + std::to_string(tolerance),       \
                        __FILE__, __LINE__);                                                \
        }                                                                                   \
    } while (false)

#define CHECK_EQ(actual, expected)                                                          \
    do {                                                                                    \
        if (!((actual) == (expected))) {                                                    \
            test::fail(std::string(#actual) + " != " + #expected, __FILE__, __LINE__);      \
        }                                                                                   \
    } while (false)
