#include "camera.hpp"
#include "myinfer.hpp"


int main(int argc, char const *argv[])
{
    K4A k4a;
    YOLO yolo;
    std::string engine_v8 = "/home/right/Infer/workspace/best.transd.engine"; 
    std::string engine_v8_seg = "/home/right/Infer/workspace/best_seg.transd.engine";

    yolo::BoxArray objs;

    pcl::PointCloud<pcl::PointXYZ> cloud_seg;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cv::Mat image_color, image_depth;
    cv::Mat image_test;

    while(1)
    {
        k4a.K4a_Image_to_Cv(image_color, image_depth);

        k4a.Cv_Depth_to_Pcl(cloud);
        k4a.K4a_Mask_to_Binary(image_test, objs);

        // yolo.Yolov8_Enable(engine_v8);
        yolo.Yolov8_Seg_Enable(engine_v8_seg);
        yolo.Single_Inference(image_color, objs);

        k4a.K4a_Color_With_Mask(image_color, objs);
        k4a.K4a_Depth_With_Mask(image_depth, objs);
        k4a.Cv_Mask_to_Pcl(cloud_seg);



        cv::imshow("Seg Depth Image", image_depth);
        cv::imshow("Seg Color Image", image_color);

        image_color.release();
        image_depth.release();

        if (cv::waitKey(1) == 27) break;







    }



    return 0;
}


