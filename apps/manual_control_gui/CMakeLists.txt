find_package(SDL2 CONFIG)

if(SDL2_FOUND)
    add_executable(manual_control_gui main.cpp)
    target_link_libraries(manual_control_gui PRIVATE tello_driver common core SDL2::SDL2)
else()
    message(STATUS "SDL2 not found - skipping manual_control_gui (install libsdl2-dev to enable)")
endif()
