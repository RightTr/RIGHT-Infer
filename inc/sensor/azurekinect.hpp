#ifndef AZUREKINECT_HPP
#define AZUREKINECT_HPP
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <k4a/k4a.hpp>

#include <iostream>
#include <unistd.h>
#include <string>

#define COUT_RED_START      std::cout << "\033[1;31m";
#define COUT_GREEN_START    std::cout << "\033[1;32m";
#define COUT_YELLOW_START   std::cout << "\033[1;33m";
#define COUT_BLUE_START     std::cout << "\033[1;34m";
#define COUT_PURPLE_START   std::cout << "\033[1;35m";
#define COUT_CYAN_START     std::cout << "\033[1;36m";
#define COUT_WHITE_START    std::cout << "\033[1;37m";
#define COUT_COLOR_END      std::cout << "\033[0m";

class K4a
{
    private:
        k4a::device device;
        k4a_device_configuration_t config;
        k4a::capture capture;
        int device_count;
        k4a::calibration k4aCalibration;
        k4a::transformation k4aTransformation;
        

    protected:
        k4a::image image_k4a_color, image_k4a_depth, image_k4a_infrared;
        k4a::image image_k4a_depth_to_color;
        k4a_calibration_camera_t depth_intrinsics;
        k4a_calibration_camera_t color_intrinsics;
        int frame_count = 0;

    public:
        bool Open();   

        void Installed_Count(); 

        void Configuration();

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