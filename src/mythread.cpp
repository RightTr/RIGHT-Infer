#include "mythread.hpp"

void* Mythread::K4a_Get_Image(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    std::shared_ptr<cv::Mat> color_show_ptr, depth_show_ptr;
    while(1)
    {   
        pthread_mutex_lock(&mutex_k4a);
        thread_instance->k4a->Image_to_Cv(*(thread_instance->color_k4a_ptr), *(thread_instance->depth_k4a_ptr));
        color_show_ptr = std::make_shared<cv::Mat>(thread_instance->color_k4a_ptr->clone());
        depth_show_ptr = std::make_shared<cv::Mat>(thread_instance->depth_k4a_ptr->clone());
        pthread_mutex_unlock(&mutex_k4a);
        if(color_show_ptr->empty())
        {
            throw("Color Image Empty");
            continue;
        }
        if(depth_show_ptr->empty())
        {
            throw("Depth Image Empty");
            continue;
        }
        cv::imshow("Color Image", *color_show_ptr);
        cv::imshow("Depth Image", *depth_show_ptr);
        cv::waitKey(10);
        usleep(100);
    }
    pthread_exit(NULL);

}


void* Mythread::K4a_Single_Inference_V8(void* argc)
{   
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    thread_instance->yolo->Yolov8_Enable(*(thread_instance->engine_v8_ptr));
    std::shared_ptr<yolo::BoxArray> objs_detect_ptr;
    std::shared_ptr<cv::Mat> color_detect_ptr;
    while(1)
    {
        pthread_mutex_lock(&mutex_k4a);
        thread_instance->yolo->Single_Inference(*(thread_instance->color_k4a_ptr), *(thread_instance->objs_ptr));
        color_detect_ptr = std::make_shared<cv::Mat>(thread_instance->color_k4a_ptr->clone());
        objs_detect_ptr = std::make_shared<yolo::BoxArray>(*(thread_instance->objs_ptr));
        pthread_mutex_unlock(&mutex_k4a);
        thread_instance->k4a->Color_With_Mask(*(color_detect_ptr), *(objs_detect_ptr));
        cv::imshow("Color Detected", *color_detect_ptr);
        cv::waitKey(10);
        usleep(100);
    }
    pthread_exit(NULL); 
}


void* Mythread::K4a_Seg_to_Pcl(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    std::shared_ptr<cv::Mat> color_seg_ptr;
    std::shared_ptr<cv::Mat> depth_seg_ptr;
    std::shared_ptr<yolo::BoxArray> objs_seg_ptr;
    thread_instance->yolo->Yolov8_Seg_Enable(*(thread_instance->engine_v8_seg_ptr));
    while(1)
    {
        pthread_mutex_lock(&mutex_k4a);
        thread_instance->yolo->Single_Inference(*(thread_instance->color_k4a_ptr), *(thread_instance->objs_ptr));
        thread_instance->k4a->Mask_to_Binary(*(thread_instance->objs_ptr));
        thread_instance->k4a->Cv_Mask_to_Pcl(*(thread_instance->cloud_seg_ptr));
        color_seg_ptr = std::make_shared<cv::Mat>(thread_instance->color_k4a_ptr->clone());
        depth_seg_ptr = std::make_shared<cv::Mat>(thread_instance->depth_k4a_ptr->clone());
        objs_seg_ptr = std::make_shared<yolo::BoxArray>(*(thread_instance->objs_ptr));
        pthread_mutex_unlock(&mutex_k4a);
        thread_instance->k4a->Color_With_Mask(*(color_seg_ptr), *(objs_seg_ptr));
        thread_instance->k4a->Depth_With_Mask(*(depth_seg_ptr), *(objs_seg_ptr));
        cv::imshow("Color Seg", *(color_seg_ptr));
        cv::imshow("Depth Seg", *(depth_seg_ptr));
        cv::waitKey(10);
        usleep(100);
    }
    pthread_exit(NULL); 
}

void* Mythread::K4a_Pcl_Process(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);

    while(1)
    {
        pthread_mutex_lock(&mutex_k4a);
        *(cloud_seg_ptr_) = *(thread_instance->cloud_seg_ptr);
        pthread_mutex_unlock(&mutex_k4a);
        // thread_instance->pclprocess->Vg_Filter(0.01, cloud_seg_ptr_);
        thread_instance->pclprocess->Sor_Filter(50, 0.01, cloud_seg_ptr_);
        thread_instance->pclprocess->Ror_Filter(35, 0.15, cloud_seg_ptr_);
        pcl::compute3DCentroid(*(cloud_seg_ptr_), *(thread_instance->centroid));

        std::cout << "x:" << thread_instance->centroid->x() 
                << ",y:" << thread_instance->centroid->y() 
                << ",z:" << thread_instance->centroid->z() << std::endl;

        pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output_opt.ply", *cloud_seg_ptr_);    
        usleep(100);
    }
    pthread_exit(NULL); 
}

void* Mythread::Rs_Get_Image(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    std::shared_ptr<cv::Mat> color_show_ptr, depth_show_ptr;
    while(1)
    {
        pthread_mutex_lock(&mutex_rs);
        thread_instance->realsense->Image_to_Cv(*(thread_instance->color_rs_ptr), *(thread_instance->depth_rs_ptr));
        color_show_ptr = std::make_shared<cv::Mat>(thread_instance->color_rs_ptr->clone());
        depth_show_ptr = std::make_shared<cv::Mat>(thread_instance->depth_rs_ptr->clone());
        pthread_mutex_unlock(&mutex_rs);
        if(color_show_ptr->empty())
        {
            throw("Color Image Empty");
            continue;
        }
        if(depth_show_ptr->empty())
        {
            throw("Depth Image Empty");
            continue;
        }
        cv::imshow("Color Image", *color_show_ptr);
        cv::imshow("Depth Image", *depth_show_ptr);
        cv::waitKey(10);
        usleep(100);
    }
    pthread_exit(NULL);
}

void* Mythread::Rs_Single_Inference_V8(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    thread_instance->yolo->Yolov8_Enable(*(thread_instance->engine_v8_ptr));
    std::shared_ptr<yolo::BoxArray> objs_detect_ptr;
    std::shared_ptr<cv::Mat> color_detect_ptr;
    while(1)
    {
        pthread_mutex_lock(&mutex_rs);
        thread_instance->yolo->Single_Inference(*(thread_instance->color_rs_ptr), *(thread_instance->objs_ptr));
        color_detect_ptr = std::make_shared<cv::Mat>(thread_instance->color_rs_ptr->clone());
        objs_detect_ptr = std::make_shared<yolo::BoxArray>(*(thread_instance->objs_ptr));
        pthread_mutex_unlock(&mutex_rs);
        thread_instance->realsense->Color_With_Mask(*(color_detect_ptr), *(objs_detect_ptr));
        cv::imshow("Color Detected", *color_detect_ptr);
        cv::waitKey(10);
        usleep(100);
    }
    pthread_exit(NULL); 
}

void* Mythread::Rs_Seg_to_Pcl(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    std::shared_ptr<cv::Mat> color_seg_ptr;
    std::shared_ptr<cv::Mat> depth_seg_ptr;
    std::shared_ptr<yolo::BoxArray> objs_seg_ptr;
    thread_instance->yolo->Yolov8_Seg_Enable(*(thread_instance->engine_v8_seg_ptr));
    while(1)
    {
        pthread_mutex_lock(&mutex_rs);
        thread_instance->yolo->Single_Inference(*(thread_instance->color_rs_ptr), *(thread_instance->objs_ptr));
        // thread_instance->k4a->Mask_to_Binary(*(thread_instance->objs_ptr));
        // thread_instance->k4a->Cv_Mask_to_Pcl(*(thread_instance->cloud_seg_ptr));
        color_seg_ptr = std::make_shared<cv::Mat>(thread_instance->color_rs_ptr->clone());
        depth_seg_ptr = std::make_shared<cv::Mat>(thread_instance->depth_rs_ptr->clone());
        objs_seg_ptr = std::make_shared<yolo::BoxArray>(*(thread_instance->objs_ptr));
        pthread_mutex_unlock(&mutex_rs);
        thread_instance->realsense->Color_With_Mask(*(color_seg_ptr), *(objs_seg_ptr));
        thread_instance->realsense->Depth_With_Mask(*(depth_seg_ptr), *(objs_seg_ptr));
        cv::imshow("Color Seg", *(color_seg_ptr));
        cv::imshow("Depth Seg", *(depth_seg_ptr));
        cv::waitKey(10);
        usleep(100);
    }
    pthread_exit(NULL); 
}

Mythread::Mythread()
{
    // k4a = new K4a;
    yolo = new Yolo;
    pclprocess = new PclProcess;
    realsense = new RealSense;
}

Mythread::~Mythread()
{
    // delete k4a;
    delete yolo;
    delete pclprocess;
    delete realsense;
}