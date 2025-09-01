cmake_minimum_required(VERSION 3.16)

project(infer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(TENSORRT_DIR "/home/right/third-party/TensorRT-8.6.1.6")

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(k4a REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)

link_directories(
    /usr/local/cuda/lib64
)

include_directories(
    inc
    ${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer/include
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_DIR}/include
    ${PCL_INCLUDE_DIRS}/include
)

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)

# add_executable(k4a_fine src/k4a_fine.cpp src/azurekinect.cpp src/myinfer.cpp 
#                     src/utils_all_in_one.cpp src/mythread.cpp
#                     src/uart.cpp src/params.cpp)

# target_link_libraries(k4a_fine 
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     k4a
#     /home/right/RIGHT-Infer/lib/libyolo.a
#     cuda cublas cudart cudnn
#     ${TENSORRT_DIR}/lib/libnvinfer.so
#     Threads::Threads
#     yaml-cpp
# )

# add_executable(rs_coarse src/rs_coarse.cpp src/realsense.cpp src/myinfer.cpp 
#                     src/utils_all_in_one.cpp src/params.cpp)

# target_link_libraries(rs_coarse 
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     /home/right/RIGHT-Infer/lib/libyolo.a
#     cuda cublas cudart cudnn
#     ${TENSORRT_DIR}/lib/libnvinfer.so
#     realsense2::realsense2
#     yaml-cpp
# )

# add_executable(rs_stereo src/rs_stereo.cpp src/realsense.cpp src/myinfer.cpp 
#                     src/utils_all_in_one.cpp src/params.cpp src/uart.cpp)

# target_link_libraries(rs_stereo 
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     /home/right/RIGHT-Infer/lib/libyolo.a
#     cuda cublas cudart cudnn
#     ${TENSORRT_DIR}/lib/libnvinfer.so
#     realsense2::realsense2
#     yaml-cpp
# )

add_executable(k4a_debug src/k4a_debug.cpp src/sensor/azurekinect_infer.cpp 
                        src/myinfer.cpp src/plnet/plnet.cpp src/sensor/azurekinect.cpp)

target_link_libraries(k4a_debug 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a
    /home/right/RIGHT-Infer/lib/libyolo.a
    cuda cublas cudart cudnn
    ${TENSORRT_DIR}/lib/libnvinfer.so
    ${TENSORRT_DIR}/lib/libnvonnxparser.so
    tensorrtbuffer
)

# add_executable(rs_debug src/rs_debug.cpp src/sensor/realsense_infer.cpp 
#                         src/myinfer.cpp src/plnet/plnet.cpp src/sensor/realsense.cpp)

# target_link_libraries(rs_debug 
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     realsense2::realsense2
#     /home/right/RIGHT-Infer/lib/libyolo.a
#     cuda cublas cudart cudnn
#     ${TENSORRT_DIR}/lib/libnvinfer.so
#     ${TENSORRT_DIR}/lib/libnvonnxparser.so
#     tensorrtbuffer
# )

add_subdirectory(src/tools tools)