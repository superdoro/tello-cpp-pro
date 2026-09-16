find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

add_library(perception STATIC
    src/depth_scaler.cpp
)

target_include_directories(perception PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# common only: perception produces common::ObstacleSnapshot and consumes
# common::SparseDepthFrame, so it never has to know that slam/ or control/
# exist. That is what lets the whole module be tested against synthetic
# depth maps with no SLAM backend and no model files.
target_link_libraries(perception PUBLIC common opencv_core PRIVATE opencv_imgproc opencv_dnn)
