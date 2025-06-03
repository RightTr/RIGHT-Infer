#ifndef CAMERA_HPP
#define CAMERA_HPP
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <k4a/k4a.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


#include <iostream>
#include <unistd.h>
#include <memory>

#include "yolo.hpp"


#define COUT_RED_START      std::cout << "\033[1;31m";
#define COUT_GREEN_START    std::cout << "\033[1;32m";
#define COUT_YELLOW_START   std::cout << "\033[1;33m";
#define COUT_BLUE_START     std::cout << "\033[1;34m";
#define COUT_PURPLE_START   std::cout << "\033[1;35m";
#define COUT_CYAN_START     std::cout << "\033[1;36m";
#define COUT_WHITE_START    std::cout << "\033[1;37m";
#define COUT_COLOR_END      std::cout << "\033[0m";

#define MIN_DISTANCE 2.0

class K4a
{
    private:
        k4a::device device;
        k4a_device_configuration_t config;
        k4a::capture capture;
        int device_count;
        k4a::image image_k4a_color, image_k4a_depth, image_k4a_infrared;
        k4a::image image_k4a_depth_to_color;
        k4a_calibration_camera_t depth_intrinsics;
        k4a_calibration_camera_t color_intrinsics;
        k4a::calibration k4aCalibration;
        k4a::transformation k4aTransformation;
        std::string output_dir = "/home/right/Datasets/Basket/";
        std::string filename;
        int frame_count = 0;

    public:
        void Open();   

        void Installed_Count(); 

        void Configuration();

        void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

        void Color_to_Cv(cv::Mat &image_cv_color);

        void Depth_to_Cv(cv::Mat &image_cv_depth);

        void Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray &objs);

        void Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray &objs);

        void Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs);

        void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        void Save_Image(int amount);

        K4a()
        {
            Installed_Count();
            Open();
            Configuration();
        }
        
        ~K4a()
        {
            image_k4a_depth.reset();
            image_k4a_color.reset();
            capture.reset();
            device.close();
        }
};

class RealSense
{
    private:
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::pipeline_profile profile;
        rs2::frameset frameset;
        cv::Mat image_rs_color, image_rs_depth;
        cv::Mat mask;

    public:
        rs2_intrinsics intrinsics_depth;
        rs2_intrinsics intrinsics_color;

        void Configuration();
        
        void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

        void Color_to_Cv(cv::Mat &image_cv_color);

        void Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs);

        void Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs);

        void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        void Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        RealSense()
        {
            Configuration();
        }

        ~RealSense()
        {

        }


};

#endif