# Shared composition helpers for the apps. Deliberately not a "library" of
# logic - it only wires existing modules together, so nothing here is
# something another module should be able to depend on.
add_library(app_support STATIC app_support.cpp)

target_include_directories(app_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(app_support PUBLIC
    common core video tello_driver slam opencv_core opencv_imgproc opencv_highgui
)
