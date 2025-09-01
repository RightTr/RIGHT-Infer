#ifndef AZUREKINECT_INFER_HPP
#define AZUREKINECT_INFER_HPP
#include "azurekinect.hpp"
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include "yolo/yolo.hpp"


class K4a_Infer : public K4a
{

    public:

        void Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray &objs);

        void Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray &objs);

        void Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs);

        void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

};

#endif