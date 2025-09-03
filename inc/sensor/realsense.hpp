#ifndef REALSENSE_HPP
#define REALSENSE_HPP
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <iostream>
#include <unistd.h>
#include <string>

#include "camerabase.hpp"


class RealSense
{
    private:
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::pipeline_profile profile;
        rs2::frameset frameset;
        cv::Mat image_rs_color, image_rs_depth;
        cv::Mat image_rs_infrared_left, image_rs_infrared_right;
        int frame_count = 0;

        ColorIntrinsics intrin_color;
        DepthIntrinsics intrin_depth;

        RealSense() = default;

        void Configuration_Default();

        void Configuration_RGBD();

        void Configuration_Infrared_Only();

    public:

        static RealSense Create_Default();

        static RealSense Create_RGBD();

        static RealSense Create_Infrared_Only();

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

        void Infrared_to_Cv(cv::Mat &image_cv_infrared_left, cv::Mat &image_cv_infrared_right);

        void Save_Image(int amount, std::string output_dir);

        ~RealSense() = default;

};

#endif