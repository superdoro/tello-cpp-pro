add_library(slam STATIC
    src/orb_slam3_localizer.cpp
    src/map_metadata.cpp
    src/camera_config.cpp
)

target_include_directories(slam PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# orb_slam3 is deliberately PRIVATE: no ORB-SLAM3, Pangolin, Sophus or Eigen
# header may reach a consumer of this library. That is what keeps the rest of
# the project on C++20 while ORB-SLAM3 compiles as C++14, and what makes
# swapping the SLAM backend a change confined to this directory.
target_link_libraries(slam
    PUBLIC common video
    PRIVATE orb_slam3
)

# This library's own sources include ORB-SLAM3 headers, which are C++14-era
# code; C++20 is fine for them, but the vendored library is built at 14.
set_target_properties(slam PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
