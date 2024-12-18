#include "mythread.hpp"





void* Mythread::K4a_Get_Image(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    while(1)
    {   
        pthread_mutex_lock(&(thread_instance->mutex));
        thread_instance->k4a->Image_to_Cv(*(thread_instance->color_ptr), *(thread_instance->depth_ptr));
        pthread_mutex_unlock(&(thread_instance->mutex));
    }

    pthread_exit(NULL);

}


void* Mythread::Single_Inference_V8(void* argc)
{   
    Mythread* thread_instance = static_cast<Mythread*>(argc);

    yolo::BoxArray* objs_detect_ptr = new yolo::BoxArray;
    cv::Mat* color_detect_ptr = new cv::Mat;
    cv::Mat* depth_detect_ptr = new cv::Mat;
    std::function<void(cv::Mat&, yolo::BoxArray&)> add_ptr = 
    std::bind(&K4a::Color_With_Mask, thread_instance->k4a, std::placeholders::_1, std::placeholders::_2);


    while(1)
    {
        pthread_mutex_lock(&(thread_instance->mutex));
        thread_instance->yolo->Yolov8_Enable(*(thread_instance->engine_v8));
        thread_instance->yolo->Single_Inference(*(thread_instance->color_ptr), *(thread_instance->objs_ptr));
        *(color_detect_ptr) = *(thread_instance->color_ptr);
        *(objs_detect_ptr) = *(thread_instance->objs_ptr);

        pthread_mutex_unlock(&(thread_instance->mutex));
        add_ptr(*color_detect_ptr, *objs_detect_ptr);
        cv::imshow("Color Detect", *color_detect_ptr);

        cv::waitKey(10);
    }
    pthread_exit(NULL); 
}


void* Mythread::Single_Inference_V8_Seg(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    
    while(1)
    {



    }
}

Mythread::Mythread()
{
    k4a = new K4a;
    yolo = new Yolo;



}

Mythread::~Mythread()
{
    delete k4a;
    delete yolo;
}