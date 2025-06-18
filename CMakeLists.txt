cmake_minimum_required(VERSION 3.16)

project(infer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
    ${TENSORRT_DIR}/include
    ${PCL_INCLUDE_DIRS}/include
)

add_executable(k4a_process src/k4a_process.cpp src/camera.cpp src/myinfer.cpp 
                    src/utils_all_in_one.cpp src/mythread.cpp
                    src/uart.cpp src/params.cpp)

target_link_libraries(k4a_process 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a
    /home/right/RIGHT-Infer/lib/libyolo.a
    cuda cublas cudart cudnn
    ${TENSORRT_DIR}/lib/libnvinfer.so
    Threads::Threads
    realsense2::realsense2
    yaml-cpp
)

add_executable(rs_process src/rs_process.cpp src/camera.cpp src/myinfer.cpp 
                    src/utils_all_in_one.cpp src/params.cpp)

target_link_libraries(rs_process 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a
    /home/right/RIGHT-Infer/lib/libyolo.a
    cuda cublas cudart cudnn
    ${TENSORRT_DIR}/lib/libnvinfer.so
    realsense2::realsense2
    yaml-cpp
)






# project(capture)

# find_package(OpenCV REQUIRED)
# find_package(k4a REQUIRED)

# add_executable(capture src/capture.cpp src/camera.cpp)

# include_directories(
#     inc
# )


# target_link_libraries(capture 
#     ${OpenCV_LIBS}
#     k4a)

# project(viewer)

# find_package(PCL 1.13 REQUIRED)

# include_directories(
#     inc
# )

# add_executable(viewer src/viewer.cpp src/pclprocess.cpp)

# target_link_libraries(viewer 
#     ${PCL_LIBRARIES}
# )
