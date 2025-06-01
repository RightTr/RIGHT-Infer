#include "mythread.hpp"

void* Mythread::K4a_Single_Inference_V8_Seg(void* argc)
{   
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    thread_instance->yolo->Yolov8_Seg_Enable(thread_instance->engine_v8_seg);
    std::shared_ptr<cv::Mat> color_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<cv::Mat> depth_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();

    while(1)
    {
        pthread_mutex_lock(&mutex_k4a_color);
        thread_instance->k4a->Image_to_Cv(*color_ptr, *depth_ptr);
        thread_instance->yolo->Single_Inference(*color_ptr, *objs_ptr);
        thread_instance->color_ptr = std::make_shared<cv::Mat>(color_ptr->clone());
        thread_instance->depth_ptr = std::make_shared<cv::Mat>(depth_ptr->clone());
        thread_instance->objs_ptr = std::make_shared<yolo::BoxArray>(*objs_ptr);
        show_ready = true;
        pthread_cond_signal(&cond_show);
        pthread_mutex_unlock(&mutex_k4a_color);

        usleep(10);
    }
    pthread_exit(NULL); 
}

void* Mythread::K4a_Image_Show(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    while (1)
    {
        pthread_mutex_lock(&mutex_k4a_color);
        while (!show_ready) 
        {
            pthread_cond_wait(&cond_show, &mutex_k4a_color); 
        }
    
        cv::Mat color = thread_instance->color_ptr->clone();
        cv::Mat depth = thread_instance->depth_ptr->clone();
        show_ready = false;
        pthread_mutex_unlock(&mutex_k4a_color);
    
        if (!color.empty()) 
        {
            cv::imshow("Color Image", color);
        }
        cv::waitKey(1);
        usleep(10);  
    }
    pthread_exit(NULL); 
}

void* Mythread::K4a_Seg_to_Pcl(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    while(1)
    {
        pthread_mutex_lock(&mutex_k4a_color);
        thread_instance->k4a->Value_Mask_to_Pcl(*(thread_instance->cloud_seg_ptr), *(thread_instance->objs_ptr));
        pthread_mutex_unlock(&mutex_k4a_color);
        usleep(10);
    }
    pthread_exit(NULL); 
}

void* Mythread::K4a_Pcl_Process(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    Eigen::VectorXf coeff;
    while(1)
    {
        pthread_mutex_lock(&mutex_k4a_color);
        *(cloud_seg_ptr_) = *(thread_instance->cloud_seg_ptr);
        pthread_mutex_unlock(&mutex_k4a_color);
        Vg_Filter(0.06, cloud_seg_ptr_);
        Sor_Filter(50, 0.01, cloud_seg_ptr_);
        usleep(1000);
    }
    pthread_exit(NULL); 
}
