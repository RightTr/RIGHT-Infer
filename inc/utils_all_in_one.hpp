#ifndef PROCESS_ALL_IN_ONE_HPP
#define PROCESS_ALL_IN_ONE_HPP

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h> 

#include <unsupported/Eigen/NonLinearOptimization>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>
#include <chrono>

#include "myinfer.hpp"
#include "uart.hpp"
#include "params.hpp"

#define TIMESTART auto Start = std::chrono::system_clock::now();
#define TIMEEND auto End = std::chrono::system_clock::now();
#define DURATION std::cout << "Duration: " << double(std::chrono::duration_cast<std::chrono::microseconds>(End - Start).count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;

static int valid = 1;

extern float ransac_dis;
extern int ransac_iters;
extern float k4a_pitch;
extern double basket_radius;
extern float k4a2robot_x;
extern float k4a2robot_y;

struct CircleFunctor
{
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud;
    double radius;
    CircleFunctor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double radius) : cloud(cloud), radius(radius){}

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
    {
        Eigen::Vector3d center(x(0), x(1), x(2));
        for (size_t i = 0; i < cloud->size(); i++)
        {
            const auto &point = (*cloud)[i];
            fvec(i) = (Eigen::Vector3d(point.x, point.y, point.z) - center).norm() - radius;
        }
        return 0;
    }

    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
    {
        Eigen::Vector3d center(x(0), x(1), x(2));

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            const auto &point = (*cloud)[i];
            Eigen::Vector3d diff(point.x - center.x(), point.y - center.y(), point.z - center.z());
            double norm = diff.norm();
            fjac(i, 0) = -diff.x() / norm;
            fjac(i, 1) = -diff.y() / norm;
            fjac(i, 2) = -diff.z() / norm;
        }

        return 0;
    }
    int inputs() const {return 3;} 
    int values() const {return cloud->size();}
};

void Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

Eigen::Vector3d FitCircle_LM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff);

void Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::Vector2f& target2d);

void Pixels_Center_Extract(const yolo::BoxArray& objs, cv::Mat& img_in, cv::Point2f& center);

void Pixels_Circle_Extract(const yolo::BoxArray& objs, cv::Mat& img_in, vector<cv::Point2f>& target2d);

class FPSCounter 
{
    public:
        FPSCounter(const std::string& name = "FPS") : name_(name), frame_count_(0) 
        {
            start_time_ = std::chrono::steady_clock::now();
        }
    
        void tick() 
        {
            frame_count_++;
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
    
            if (duration >= 1) 
            {
                double fps = frame_count_ / static_cast<double>(duration);
                COUT_CYAN_START
                std::cout << "[" << name_ << "] FPS: " << fps << std::endl;
                COUT_COLOR_END
                frame_count_ = 0;
                start_time_ = now;
            }
        }
    
    private:
        std::string name_;
        int frame_count_;
        std::chrono::steady_clock::time_point start_time_;
    };

#endif