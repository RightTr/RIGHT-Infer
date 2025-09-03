#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include "yolo/yolo.hpp"
#include "yolo/myinfer.hpp"

template<typename CameraType>
class CameraWrapper 
{
    public:
        CameraWrapper(CameraType& cam) : camera(cam) {}
        
        void Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray &objs);

        void Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray &objs);

        void Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs);

        void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

        void Draw_Lines(cv::Mat &image_cv_color, std::vector<Eigen::Vector4d> lines);

    private:
        CameraType& camera;
};
