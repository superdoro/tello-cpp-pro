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
add_subdirectory(video)
add_subdirectory(control)
add_subdirectory(perception)

# Milestone (a)/(b): no SLAM needed.
add_subdirectory(apps/manual_control)
add_subdirectory(apps/manual_control_gui)
add_subdirectory(apps/video_view)

# Milestones (c)-(f): mapping, localization and autonomous flight all need
# ORB-SLAM3, which is a heavy build, so they are opt-in.
if(BUILD_SLAM)
    add_subdirectory(third_party)
endif()

# Always added: slam/ defines an interface-only target (headers, no ORB-SLAM3)
# that mocks and tests use, and gates the implementation on BUILD_SLAM itself.
add_subdirectory(slam)

if(BUILD_SLAM)
    add_subdirectory(apps/support)
    add_subdirectory(apps/map_builder)
    add_subdirectory(apps/relocalize)
    add_subdirectory(apps/autonomous_mission)
    if(BUILD_TOOLS)
        add_subdirectory(apps/tools/calibrate_camera)
        add_subdirectory(apps/tools/calibrate_scale)
        add_subdirectory(apps/tools/trajectory_to_mission)
    endif()
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
