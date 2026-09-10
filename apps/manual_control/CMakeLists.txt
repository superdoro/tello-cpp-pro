add_executable(manual_control main.cpp)
target_link_libraries(manual_control PRIVATE tello_driver common core)
