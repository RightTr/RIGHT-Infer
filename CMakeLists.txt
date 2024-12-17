cmake_minimum_required(VERSION 3.16)

project(infer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(TENSORRT_DIR "/home/right/third-party/TensorRT-8.6.1.6")


find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(k4a REQUIRED)
find_package(PCL 1.13 REQUIRED)


include_directories(
    inc
    ${TENSORRT_DIR}/include
    ${PCL_INCLUDE_DIRS}/include
)




add_executable(infer src/main.cpp src/camera.cpp src/myinfer.cpp src/pclprocess.cpp src/queue.cpp)


target_link_libraries(infer 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    k4a
    /home/right/RIGHT-Infer/lib/libyolo.a
    cuda cublas cudart cudnn
    ${TENSORRT_DIR}/lib/libnvinfer.so
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
