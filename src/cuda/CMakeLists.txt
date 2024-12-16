cmake_minimum_required(VERSION 3.12)

project(cuda)

set(TENSORRT_DIR "/home/right/third-party/TensorRT-8.6.1.6")

file(GLOB_RECURSE YOLO_SOURCES "*.cu")
message(STATUS "YOLO_SOURCES: ${YOLO_SOURCES}")

find_package(CUDA REQUIRED)

cuda_add_library(yolo STATIC ${YOLO_SOURCES})


include_directories(
    ${TENSORRT_DIR}/include
)

target_link_libraries(yolo 
    ${OpenCV_LIBS}
)
