add_executable(map_builder main.cpp)
target_link_libraries(map_builder PRIVATE slam video common opencv_highgui opencv_imgproc)
