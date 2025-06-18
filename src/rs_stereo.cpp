#include "camera.hpp"
#include "myinfer.hpp"
#include "utils_all_in_one.hpp"
#include "tcp_socket.hpp"
#include "params.hpp"
#include <opencv2/photo.hpp> 

#define PI 3.1415926535

int main(int argc, char const *argv[])
{
    Yolo yolo;
    RealSense rs = RealSense::Create_Infrared_Only();
    string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket_ir/best.engine";
    yolo::BoxArray objs_left, objs_right;
    vector<float> center_left(2), center_right(2);
    float disparity = 0;
    float depth = 0;
    float target_x = 0, target_z = 0;
    FPSCounter fps("Realsense Stream");

    cv::Mat image_infrared_left, image_infrared_right;

    float fx = rs.intrinsics_infrared.fx;
    float cx = rs.intrinsics_infrared.ppx;

    while (1)
    {
        rs.Infrared_to_Cv(image_infrared_left, image_infrared_right);

        cv::detailEnhance(image_infrared_left, image_infrared_left, 15, 0.15);
        cv::detailEnhance(image_infrared_right, image_infrared_right, 15, 0.15);

        yolo.Yolov8_Seg_Enable(engine_v8_seg);
        yolo.Single_Inference(image_infrared_left, objs_left);
        yolo.Single_Inference(image_infrared_right, objs_right);
        
        rs.Color_With_Mask(image_infrared_left, objs_left);
        rs.Color_With_Mask(image_infrared_right, objs_right);

        Pixels_Center_Extract(objs_left, image_infrared_left, center_left);
        Pixels_Center_Extract(objs_right, image_infrared_right, center_right);
        
        disparity = center_left[0] - center_right[0];
        depth = (fx * 0.059) / disparity; 

        target_x = (center_left[0] - cx) * depth / fx;
        target_z = depth;

        cout << "Disparity: " << disparity << ", Depth: " << depth 
             << ", Target X: " << target_x << ", Target Z: " << target_z << endl;


        cv::imshow("Infrared Left Image", image_infrared_left); 
        cv::imshow("Infrared Right Image", image_infrared_right);
        if (cv::waitKey(1) == 27)
            break;
    }
    return 0;
}


