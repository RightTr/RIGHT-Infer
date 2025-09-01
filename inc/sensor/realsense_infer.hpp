#ifndef REALSENSE_INFER_HPP
#define REALSENSE_INFER_HPP

#include "realsense.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include "yolo/yolo.hpp"


class RealSense_Infer : public RealSense
{
    public:

        void Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs);

        void Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs);

        void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        void Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

};

#endif