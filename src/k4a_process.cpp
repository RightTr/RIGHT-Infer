#include "camera.hpp"
#include "myinfer.hpp"
#include "utils_all_in_one.hpp"
#include "mythread.hpp"

// int main(int argc, char const *argv[])
// {
//     K4a k4a;
//     Yolo yolo;
//     std::string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket/best.engine";
//     yolo::BoxArray objs;
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     cv::Mat image_color, image_depth;
//     int index = 0;

//     while(1)
//     {   
//         k4a.Image_to_Cv(image_color, image_depth);
//         yolo.Yolov8_Seg_Enable(engine_v8_seg);

//         yolo.Single_Inference(image_color, objs);
//         TIMESTART
//         k4a.Value_Mask_to_Pcl(cloud, objs);
//         TIMEEND
//         DURATION
//         // if(index < 500)
//         // {
//         //     pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/datasets/ply/test/basket" + std::to_string(index++) + ".ply", cloud);
//         //     std::cout << "Basket" << index << " Save Success!" << std::endl; 
//         //     usleep(1000);
//         // }
//         // else
//         // {
//         //     std::cout << "Pointcloud Save Over!" << std::endl; 
//         // }
//         k4a.Color_With_Mask(image_color, objs);
//         k4a.Depth_With_Mask(image_depth, objs);     

//         cv::imshow("Seg Depth Image", image_depth);
//         cv::imshow("Seg Color Image", image_color);
//         if (cv::waitKey(1) == 27) break;

//     }
//     return 0;
// }

int main(int argc, char const *argv[])
{
    K4a_Read_Parameters("/home/right/RIGHT-Infer/config/config.yaml");

    Mythread mythread;

    pthread_t thread[8] = {0};

    pthread_create(&thread[0], NULL, Mythread::K4a_Single_Inference_V8_Seg, &mythread);
    pthread_create(&thread[1], NULL, Mythread::K4a_Image_Show, &mythread);
    pthread_create(&thread[2], NULL, Mythread::TCP_Server, &mythread);
    pthread_create(&thread[3], NULL, Mythread::Pcl_Process, &mythread);

    for(int i = 0; i < 8; i++)
    {
        pthread_join(thread[i], NULL);
    }

    return 0;
}



