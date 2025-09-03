#ifndef AZUREKINECT_HPP
#define AZUREKINECT_HPP
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <k4a/k4a.hpp>

#include <iostream>
#include <unistd.h>
#include <string>

#include "camerabase.hpp"

class K4a
{
    private:
        k4a::device device;
        k4a_device_configuration_t config;
        k4a::capture capture;
        int device_count;
        k4a::calibration k4aCalibration;
        k4a::transformation k4aTransformation;
        ColorIntrinsics intrin_color;
        DepthIntrinsics intrin_depth; 
        int frame_count = 0;
        k4a::image image_k4a_color, image_k4a_depth, image_k4a_infrared;
        k4a::image image_k4a_depth_to_color;

        bool Open();   

        void Installed_Count(); 

        void Configuration();
        
    public:

        inline const ColorIntrinsics& get_ColorIntrinsics() const noexcept
        {
            return intrin_color;
        }

        inline const DepthIntrinsics& get_DepthIntrinsics() const noexcept
        {
            return intrin_depth;
        }

        void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

        void Color_to_Cv(cv::Mat &image_cv_color);

        void Depth_to_Cv(cv::Mat &image_cv_depth);

        void Save_Image(int amount, std::string output_dir);

        K4a()
        {
            Installed_Count();
            if(Open())
            {
                Configuration();
            }
        }
        
        ~K4a()
        {
            image_k4a_depth.reset();
            image_k4a_color.reset();
            capture.reset();
            device.close();
        }
};

#endif