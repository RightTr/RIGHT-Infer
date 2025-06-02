#ifndef THREAD_HPP
#define THREAD_HPP

#include <pthread.h>
#include <memory>
#include <functional>
#include <chrono>
#include "camera.hpp"
#include "myinfer.hpp"
#include "process_all_in_one.hpp"
#include "tcp_socket.hpp"

using namespace std;
static pthread_mutex_t mutex_k4a_color = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond_show = PTHREAD_COND_INITIALIZER;
static bool show_ready = false;
static bool depth_request = false;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 

class Mythread
{
    private:
        K4a* k4a;
        Yolo* yolo;
        string engine_v8 = "/home/right/RIGHT-Infer/workspace/best.engine"; 
        string engine_v8_seg= "/home/right/RIGHT-Infer/workspace/Basket/best.engine"; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 
        std::shared_ptr<Eigen::Vector4f> centroid = std::make_shared<Eigen::Vector4f>();
        std::shared_ptr<cv::Mat> color_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<cv::Mat> depth_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();

        string ip = "127.0.0.1";
        uint16_t port = 8888;
        TcpSocket tcpsocket_list;

    public: 

        static void* K4a_Single_Inference_V8_Seg(void* argc); 

        static void* K4a_Image_Show(void* argc);

        static void* K4a_Depth_Get(void* argc);

        static void* K4a_Seg_to_Pcl(void* argc);

        static void* K4a_Pcl_Process(void* argc);

        static void* TCP_Server(void* argc);

        static void* TCP_Client_Handler(void* argc);

        Mythread()
        {
            // k4a = new K4a;
            yolo= new Yolo;
            if (!tcpsocket_list.Socket())
                throw runtime_error("Socket failed");
            if (!tcpsocket_list.Bind(&ip, port))
                throw runtime_error("Bind failed");
            if (!tcpsocket_list.Listen())
                throw runtime_error("Listen failed");
        }

        ~Mythread()
        {
            // delete k4a;
            delete yolo;
        }


};



#endif