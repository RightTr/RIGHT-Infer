cmake_minimum_required(VERSION 3.16)

project(tools)


add_executable(k4a_rgb_cap k4a_rgb_cap.cpp ../sensor/azurekinect.cpp)

target_link_libraries(k4a_rgb_cap 
    ${OpenCV_LIBS}
    k4a
)


add_executable(rs_ir_cap rs_ir_cap.cpp ../sensor/realsense.cpp)

target_link_libraries(rs_ir_cap 
    ${OpenCV_LIBS}
    realsense2::realsense2
)
