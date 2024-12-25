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

static pthread_mutex_t mutex_k4a = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_rs = PTHREAD_MUTEX_INITIALIZER;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 

class Mythread
{
    private:
        K4a* k4a;
        Yolo* yolo;
        PclProcess* pclprocess;
        RealSense* realsense;

        std::shared_ptr<std::string> engine_v8_ptr = std::make_shared<std::string>("/home/right/RIGHT-Infer/workspace/best.transd.engine"); 
        std::shared_ptr<std::string> engine_v8_seg_ptr = std::make_shared<std::string>("/home/right/RIGHT-Infer/workspace/best_seg.transd.engine"); 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 
        std::shared_ptr<Eigen::Vector4f> centroid = std::make_shared<Eigen::Vector4f>();
        std::shared_ptr<cv::Mat> color_k4a_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<cv::Mat> depth_k4a_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<cv::Mat> color_rs_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<cv::Mat> depth_rs_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();



    public: 

        static void* K4a_Get_Image(void* argc);

        static void* Rs_Get_Image(void* argc);

        static void* K4a_Single_Inference_V8(void* argc); 

        static void* K4a_Mask_Seg_to_Pcl(void* argc);

        static void* K4a_Pcl_Process(void* argc);

        Mythread();

        ~Mythread();


};



#endif