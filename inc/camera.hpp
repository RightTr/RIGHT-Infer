#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <k4a/k4a.hpp>

#include <iostream>
#include <unistd.h>

#include "yolo.hpp"








#define COUT_RED_START      std::cout << "\033[1;31m";
#define COUT_GREEN_START    std::cout << "\033[1;32m";
#define COUT_YELLOW_START   std::cout << "\033[1;33m";
#define COUT_BLUE_START     std::cout << "\033[1;34m";
#define COUT_PURPLE_START   std::cout << "\033[1;35m";
#define COUT_CYAN_START     std::cout << "\033[1;36m";
#define COUT_WHITE_START    std::cout << "\033[1;37m";
#define COUT_COLOR_END      std::cout << "\033[0m";

class K4A
{
    private:
        k4a::device device;
        k4a_device_configuration_t config;
        k4a::capture capture;
        int device_count;
        k4a::image image_k4a_color, image_k4a_depth, image_k4a_infrared;
        k4a::image image_k4a_depth_to_color, image_k4a_depth_to_pcl;

        k4a_calibration_t calibration_depth;

        k4a::calibration k4aCalibration;

        k4a::transformation k4aTransformation;
        
        std::string output_dir = "/home/right/Datasets/Basket/";

        std::string filename;

        int frame_count = 0;

        cv::Mat mask, mask_color, mask_depth;
        cv::Mat image_mask_binary;

        cv::Mat image_cv_xyz;

        pcl::PointCloud<pcl::PointXYZ> cloud_seg;


    public:

        

        void K4a_Open();   

        void K4a_Installed_Count(); 

        void K4a_Configuration();

        void K4a_Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

        void K4a_Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs);

        void K4a_Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs);

        void K4a_Mask_to_Binary(cv::Mat &image_cv_binary, yolo::BoxArray objs);

        void Cv_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        void K4a_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        void Cv_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud); 

        void K4a_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud); 

        void K4a_Save_Image(int amount);

        K4A();
        
        ~K4A();
};