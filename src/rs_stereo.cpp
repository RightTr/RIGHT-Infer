#include "camera.hpp"
#include "myinfer.hpp"
#include "utils_all_in_one.hpp"
#include "uart.hpp"
#include <opencv2/photo.hpp> 

#define PI 3.1415926535

int main(int argc, char const *argv[])
{
    Yolo yolo;
    RealSense rs = RealSense::Create_Infrared_Only();
    com::UART uart;
    string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket_ir/best.engine";
    yolo::BoxArray objs_left, objs_right;
    vector<float> center_left(2), center_right(2);
    float disparity = 0;
    float depth = 0;
    float target2left_x= 0, target2left_z = 0;
    float target2cam_x = 0, target2cam_z = 0;
    float target2robot_x = 0, target2robot_z = 0;
    float angle = 0;
    FPSCounter fps("Realsense Stream");

    uint8_t buf[12] = {0};
    buf[0] = 0xFF; 
    buf[1] = 0xFE;
    buf[10] = 0xAA;
    buf[11] = 0xDD;

    cv::Mat image_infrared_left, image_infrared_right;
    cv::Mat temp;

    float fx = rs.intrinsics_infrared.fx;
    float cx = rs.intrinsics_infrared.ppx;

    yolo.Yolov8_Seg_Enable(engine_v8_seg);

    while (1)
    {
        rs.Infrared_to_Cv(image_infrared_left, image_infrared_right);

        cv::detailEnhance(image_infrared_left, image_infrared_left, 15, 0.15);
        cv::detailEnhance(image_infrared_right, image_infrared_right, 15, 0.15);

        yolo.Single_Inference(image_infrared_left, objs_left);
        yolo.Single_Inference(image_infrared_right, objs_right);
        
        rs.Color_With_Mask(image_infrared_left, objs_left);
        rs.Color_With_Mask(image_infrared_right, objs_right);

        Pixels_Center_Extract(objs_left, image_infrared_left, center_left);
        Pixels_Center_Extract(objs_right, image_infrared_right, center_right);
        
        disparity = center_left[0] - center_right[0];
        depth = (fx * 0.059) / disparity; 
        
        target2left_x = (center_left[0] - cx) * depth / fx;
        target2left_z = depth;
        target2cam_x = target2left_x - 0.0115;
        target2cam_z = target2left_z;
        target2robot_x = target2cam_x;
        target2robot_z = target2cam_z * sinf(33.0 * PI / 180.0) - 0.33;

        memcpy(buf + 2, &target2robot_x, sizeof(target2robot_x));
        memcpy(buf + 6, &target2robot_z, sizeof(target2robot_z));
        uart.UART_SEND(buf, sizeof(buf));

        cout << "disparity: " << disparity << ", depth: " << depth 
             << ", target2robot_x: " << target2robot_x << ", target2robot_z: " << target2robot_z << endl;
        fps.tick();
        cv::imshow("Infrared Left Image", image_infrared_left); 
        cv::imshow("Infrared Right Image", image_infrared_right);
        if (cv::waitKey(1) == 27)
            break;
    }
    return 0;
}


