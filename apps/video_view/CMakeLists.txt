add_executable(video_view main.cpp)
target_link_libraries(video_view PRIVATE video tello_driver common core opencv_highgui opencv_imgproc)
