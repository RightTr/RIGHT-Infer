#include "camera.hpp"
#include "myinfer.hpp"
#include "process_all_in_one.hpp"

int main(int argc, char const *argv[])
{
    Yolo yolo;
    RealSense rs;
    std::string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket/best.engine";
    yolo::BoxArray objs;
    cv::Mat image_color;
    cv::Point2f center;

    while (1)
    {
        rs.Color_to_Cv(image_color);
        yolo.Yolov8_Seg_Enable(engine_v8_seg);
        yolo.Single_Inference(image_color, objs);
        rs.Color_With_Mask(image_color, objs);
        Pixels_Center_Extract(objs, image_color, center);
        cv::imshow("Seg Color Image", image_color);
        if (cv::waitKey(1) == 27)
            break;
    }
    return 0;
}
