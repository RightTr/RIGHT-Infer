#include "mythread.hpp"

void* Mythread::K4a_Single_Inference_V8_Seg(void* argc)
{   
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    thread_instance->yolo->Yolov8_Seg_Enable(thread_instance->engine_v8_seg);
    std::shared_ptr<cv::Mat> color_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<cv::Mat> depth_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 

    pthread_mutex_lock(&mutex_signal_shared);
    while(1)
    {
        while (!align_signal_shared) 
        {
            pthread_cond_wait(&cond_aligned, &mutex_signal_shared);
        }
        pthread_mutex_unlock(&mutex_signal_shared);

        thread_instance->k4a->Image_to_Cv(*color_ptr, *depth_ptr);
        thread_instance->yolo->Single_Inference(*color_ptr, *objs_ptr);
        thread_instance->k4a->Color_With_Mask(*color_ptr, *objs_ptr);
        thread_instance->k4a->Value_Mask_to_Pcl(*cloud_seg_ptr, *objs_ptr);

        if(pthread_mutex_trylock(&mutex_k4a_show) == 0) 
        {
            thread_instance->color_ptr = std::make_shared<cv::Mat>(color_ptr->clone());
            show_ready = true;
            pthread_cond_signal(&cond_show);
            pthread_mutex_unlock(&mutex_k4a_show);
        }

        pthread_mutex_lock(&mutex_pcl);
        thread_instance->cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud_seg_ptr);
        pthread_mutex_unlock(&mutex_pcl);
        
        usleep(10);
        pthread_mutex_lock(&mutex_signal_shared);
    }
    pthread_mutex_unlock(&mutex_signal_shared);
    pthread_exit(NULL);
}

void* Mythread::K4a_Image_Show(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    pthread_mutex_lock(&mutex_k4a_show);
    while (1)
    {
        while (!show_ready) 
        {
            pthread_cond_wait(&cond_show, &mutex_k4a_show); 
        }
    
        cv::Mat color = thread_instance->color_ptr->clone();
        show_ready = false;
        pthread_mutex_unlock(&mutex_k4a_show);
    
        if (!color.empty()) 
        {
            cv::imshow("Color Image", color);
        }
        cv::waitKey(1);
        usleep(10);  
        pthread_mutex_lock(&mutex_k4a_show);
    }
    pthread_mutex_unlock(&mutex_k4a_show);
    pthread_exit(NULL); 
}

void* Mythread::Pcl_Process(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); 
    Eigen::Vector2f target2d;

    pthread_mutex_lock(&mutex_signal_shared);
    while(1)
    {
        while (!align_signal_shared) 
        {
            pthread_cond_wait(&cond_aligned, &mutex_signal_shared);
        }
        usleep(100);
        pthread_mutex_unlock(&mutex_signal_shared);
        pthread_mutex_lock(&mutex_pcl);
        *(cloud_seg_ptr) = *(thread_instance->cloud_seg_ptr);
        pthread_mutex_unlock(&mutex_pcl);
        Vg_Filter(vg_leafsize, cloud_seg_ptr);
        Sor_Filter(sor_amount, sor_dis, cloud_seg_ptr);
        Circle_Extract(cloud_seg_ptr, target2d);
        usleep(10);
        pthread_mutex_lock(&mutex_signal_shared);
    }
    pthread_mutex_unlock(&mutex_signal_shared);
    pthread_exit(NULL); 
}

void* Mythread::TCP_Server(void* argc)
{
    Mythread* thread_instance = static_cast<Mythread*>(argc);
    uint8_t buf[5] = {0};
    float pixel_diffx = 0.;
    bool align_signal = 0;
    while (1)
    {
        TcpSocket* new_tcpsocket = new TcpSocket();
        if(!thread_instance->tcpsocket_list.Accept(new_tcpsocket))
        {
            delete new_tcpsocket;
            continue; 
        }
        else
        {
            COUT_GREEN_START
            cout << "Connection built!" << endl;
            COUT_COLOR_END
        }
        pthread_t tid;
        pthread_create(&tid, nullptr, TCP_Client_Rs_Handler, new_tcpsocket);
        pthread_detach(tid); 
    }
    pthread_exit(NULL); 
}

void* Mythread::TCP_Client_Rs_Handler(void* argc)
{
    TcpSocket* socket = static_cast<TcpSocket*>(argc);
    uint8_t buf[5] = {0};
    float pixel_diffx = 0.;
    bool align_signal = 0;
    while (1)
    {
        if (!socket->Receive(buf)) 
        {
            COUT_RED_START
            cout << "Client disconnected, exiting thread!" << endl;
            align_signal = 0;
            COUT_COLOR_END
            break; 
        }
        memcpy(&align_signal, buf, 1);
        memcpy(&pixel_diffx, buf + 1, 4);

        pthread_mutex_lock(&mutex_signal_shared);
        align_signal_shared = (buf[0] == 1);
        if (align_signal_shared) 
        {
            pthread_cond_broadcast(&cond_aligned); 
            COUT_YELLOW_START
            cout << "Wake up all fine worker threads!" << endl;
            COUT_COLOR_END
        } 
        else 
        {
            COUT_YELLOW_START
            cout << "Fine workers will wait." << endl;
            COUT_COLOR_END
        }
        pthread_mutex_unlock(&mutex_signal_shared);

        memset(buf, 0, sizeof(buf));
        cout << "Client_rs Say: align_signal(" << align_signal << 
                ") pixel_diffx(" << pixel_diffx << ")"<< endl;
    }
    socket->Close();
    delete socket;
    pthread_exit(NULL); 
}

void* Mythread::UART_Handler(void* argc)
{

    while(1)
    {

    }
}