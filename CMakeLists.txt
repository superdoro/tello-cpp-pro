cmake_minimum_required(VERSION 3.16)
project(tello_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(BUILD_TESTS "Build unit tests" OFF)
option(BUILD_TOOLS "Build calibration/record/replay tools" OFF)
option(BUILD_SLAM "Build the ORB-SLAM3 wrapper and dependent apps (requires third_party/ORB_SLAM3)" OFF)

# Foundation layers, in dependency order.
add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(drivers/tello)

# Milestone (a): Tello connectivity + manual control. video/, slam/, control/
# and their apps are added here as they're implemented in later milestones.
add_subdirectory(apps/manual_control)

if(BUILD_SLAM)
    add_subdirectory(third_party)
    add_subdirectory(slam)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
