add_library(tello_driver STATIC
    src/tello_driver.cpp
    src/tello_state_parser.cpp
)

target_include_directories(tello_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(tello_driver PUBLIC common core)
