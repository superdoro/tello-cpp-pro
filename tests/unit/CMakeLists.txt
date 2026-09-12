# Plain executables driven by CTest - see test_check.hpp for why there is no
# test framework dependency here.
function(add_unit_test name)
    add_executable(${name} ${name}.cpp ${ARGN})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${name} PRIVATE common)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_unit_test(test_frame_alignment)
target_link_libraries(test_frame_alignment PRIVATE control)

add_unit_test(test_pid_flight_controller)
target_link_libraries(test_pid_flight_controller PRIVATE control)

add_unit_test(test_mission_planner)
target_link_libraries(test_mission_planner PRIVATE control)

add_unit_test(test_queues)
add_unit_test(test_udp_socket)

add_unit_test(test_h264_decoder)
target_link_libraries(test_h264_decoder PRIVATE video opencv_videoio opencv_imgproc)

# Reaches into drivers/tello/src on purpose: the state parser is an internal
# detail of the driver, not part of its public interface, but it is pure
# string handling and exactly the sort of thing that should be tested.
add_unit_test(test_tello_state_parser ../../drivers/tello/src/tello_state_parser.cpp)
