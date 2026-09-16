find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio calib3d)

add_executable(make_synthetic_video make_synthetic_video.cpp)
target_link_libraries(make_synthetic_video PRIVATE common opencv_core opencv_imgproc
                                                    opencv_videoio opencv_calib3d)

# Drives the control stack from recorded poses, with no SLAM in the loop, so
# its output is deterministic and two runs are directly comparable. Needs no
# SLAM build at all.
add_executable(replay_flight replay_flight.cpp)
target_link_libraries(replay_flight PRIVATE common control)

# Only meaningful alongside the SLAM pipeline it evaluates.
if(BUILD_SLAM)
    find_package(Eigen3 3.1.0 REQUIRED)
    add_executable(compare_trajectory compare_trajectory.cpp)
    target_link_libraries(compare_trajectory PRIVATE common slam)
    target_include_directories(compare_trajectory SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()
