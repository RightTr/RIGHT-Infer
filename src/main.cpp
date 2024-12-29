#include "camera.hpp"
#include "myinfer.hpp"
#include "pclprocess.hpp"
#include "queue.hpp"
#include "mythread.hpp"



// int main(int argc, char const *argv[])
// {
//     // K4a k4a;
//     Yolo yolo;
//     RealSense realsense;
//     // std::string engine_v8 = "/home/right/RIGHT-Infer/workspace/best.transd.engine"; 
//     std::string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/best_seg.transd.engine";

//     yolo::BoxArray objs;

//     // pcl::PointCloud<pcl::PointXYZ> cloud_seg;
//     // pcl::PointCloud<pcl::PointXYZ> cloud;

//     cv::Mat image_color, image_depth;


//     while(1)
//     {
//         // realsense.Image_to_Cv(image_color, image_depth);
//         realsense.Image_to_Cv(image_color, image_depth);

//         // // k4a.K4a_Depth_to_Pcl(cloud);
//         // // k4a.Cv_Depth_to_Pcl(cloud);
//         // // k4a.Mask_to_Binary(image_test, objs);
       

//         // // yolo.Yolov8_Enable(engine_v8);
//         yolo.Yolov8_Seg_Enable(engine_v8_seg);
//         yolo.Single_Inference(image_color, objs);
//         realsense.Color_With_Mask(image_color, objs);
//         realsense.Depth_With_Mask(image_depth, objs);

//         // k4a.Color_With_Mask(image_color, objs);
//         // k4a.Depth_With_Mask(image_depth, objs);
//         // // k4a.Cv_Mask_to_Pcl(cloud_seg);

//         cv::imshow("Seg Depth Image", image_depth);
//         cv::imshow("Seg Color Image", image_color);

//         // image_color.release();
//         // image_depth.release();

//         if (cv::waitKey(1) == 27) break;

//     }

//     return 0;
// }

int main(int argc, char const *argv[])
{
    Mythread mythread;

    pthread_t thread[8] = {0};

    // pthread_create(&thread[0], NULL, Mythread::K4a_Get_Image, &mythread);
    // pthread_create(&thread[1], NULL, Mythread::K4a_Seg_to_Pcl, &mythread);
    // pthread_create(&thread[2], NULL, Mythread::K4a_Pcl_Process, &mythread);
    pthread_create(&thread[4], NULL, Mythread::Rs_Get_Image, &mythread);
    // pthread_create(&thread[5], NULL, Mythread::Rs_Single_Inference_V8, &mythread);
    pthread_create(&thread[6], NULL, Mythread::Rs_Seg_to_Pcl, &mythread);


    for(int i = 0; i < 8; i++)
    {
        pthread_join(thread[i], NULL);
    }


    // pthread_join(thread[0], NULL);
    // pthread_join(thread[1], NULL);
    // pthread_join(thread[2], NULL);
    // pthread_join(thread[4], NULL);
    // pthread_join(thread[5], NULL);
    return 0;
}



