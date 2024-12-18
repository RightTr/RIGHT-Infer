#ifndef THREAD_HPP
#define THREAD_HPP

#include <pthread.h>
#include <memory>
#include <functional>
#include "camera.hpp"
#include "myinfer.hpp"
#include "pclprocess.hpp"


class Mythread
{
    private:
        K4a* k4a;
        Yolo* yolo;

        std::shared_ptr<std::string> engine_v8 = std::make_shared<std::string>("/home/right/RIGHT-Infer/workspace/best.transd.engine"); 
        std::shared_ptr<std::string> engine_v8_seg = std::make_shared<std::string>("/home/right/RIGHT-Infer/workspace/best_seg.transd.engine"); 

        pthread_mutex_t mutex;

        std::shared_ptr<cv::Mat> color_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<cv::Mat> depth_ptr = std::make_shared<cv::Mat>();
        std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();



    public: 

        static void* K4a_Get_Image(void* argc);

        static void* Single_Inference_V8(void* argc); 

        static void* Single_Inference_V8_Seg(void* argc);

        Mythread();

        ~Mythread();


};



#endif