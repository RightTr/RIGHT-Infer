#ifndef __MYINFER_HPP__
#define __MYINFER_HPP__

#include <opencv2/opencv.hpp>

#include "cpm.hpp"
#include "infer.hpp"
#include "yolo.hpp"

#include <chrono>


// void batch_inference();
// void perf();

static const char *labels[] = {
    "Basket"
};

class Yolo
{
    private:
        std::string engine;
        yolo::Type type;
        yolo::Image cvimg(const cv::Mat &image);
        bool load_flag = 0;

    public:   
        

        void Yolov8_Enable(std::string &engine_);

        void Yolov8_Seg_Enable(std::string &engine_seg);

        void Single_Inference(std::string path);

        void Single_Inference(cv::Mat &image);

        void Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out);

        Yolo();

        ~Yolo();

};



#endif
