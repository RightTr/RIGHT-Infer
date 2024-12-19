#ifndef THREAD_HPP
#define THREAD_HPP

#include <pthread.h>
#include <memory>
#include <functional>
#include <chrono>
#include "camera.hpp"
#include "myinfer.hpp"
#include "pclprocess.hpp"
#include "queue.hpp"

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 

class Mythread
{
    private:
        K4a* k4a;
        Yolo* yolo;
        PclProcess* pclprocess;

        std::shared_ptr<std::string> engine_v8_ptr = std::make_shared<std::string>("/home/right/RIGHT-Infer/workspace/best.transd.engine"); 
        std::shared_ptr<std::string> engine_v8_seg_ptr = std::make_shared<std::string>("/home/right/RIGHT-Infer/workspace/best_seg.transd.engine"); 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        

        std::shared_ptr<cv::Mat> color_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<cv::Mat> depth_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();



    public: 

        static void* K4a_Get_Image(void* argc);

        static void* Single_Inference_V8(void* argc); 

        static void* Mask_Seg_to_Pcl(void* argc);

        static void* Pcl_Process(void* argc);

        Mythread();

        ~Mythread();


};



#endif