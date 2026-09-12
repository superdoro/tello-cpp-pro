find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

add_library(video STATIC
    src/h264_decoder.cpp
    src/video_file_source.cpp
    src/tello_video_source.cpp
)

target_include_directories(video PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(video
    PUBLIC common opencv_core opencv_imgproc opencv_videoio
    PRIVATE PkgConfig::FFMPEG
)
