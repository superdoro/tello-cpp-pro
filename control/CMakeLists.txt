find_package(OpenCV REQUIRED COMPONENTS core)

add_library(control STATIC
    src/frame_alignment.cpp
    src/pid_flight_controller.cpp
    src/mission_planner.cpp
    src/mission_io.cpp
)

target_include_directories(control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# No dependency on slam/ or drivers/: the controller consumes plain
# common:: types and emits plain common:: types, which is what lets the whole
# safety-critical part of the system be unit-tested with no hardware and no
# SLAM backend. OpenCV is here only as a YAML reader for route files.
target_link_libraries(control PUBLIC common PRIVATE opencv_core)
