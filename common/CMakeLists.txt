add_library(common STATIC
    src/logging.cpp
    src/udp_socket.cpp
)

target_include_directories(common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)
target_link_libraries(common PUBLIC Threads::Threads)
