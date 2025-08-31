cmake_minimum_required(VERSION 3.16)

project(tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

add_executable(k4a_rgb_cap k4a_rgb_cap.cpp ../azurekinect.cpp)

target_link_libraries(k4a_rgb_cap 
    ${OpenCV_LIBS}
    /home/right/RIGHT-Infer/lib/libyolo.a
    cuda cublas cudart cudnn
    ${TENSORRT_DIR}/lib/libnvinfer.so
    k4a
)


add_executable(rs_ir_cap rs_ir_cap.cpp ../realsense.cpp)

target_link_libraries(rs_ir_cap 
    ${OpenCV_LIBS}
    /home/right/RIGHT-Infer/lib/libyolo.a
    cuda cublas cudart cudnn
    ${TENSORRT_DIR}/lib/libnvinfer.so
    realsense2::realsense2
)
