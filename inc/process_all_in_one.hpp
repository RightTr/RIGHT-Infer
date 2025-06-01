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

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>
#include <chrono>

#include "myinfer.hpp"

#define TIMESTART auto Start = std::chrono::system_clock::now();
#define TIMEEND auto End = std::chrono::system_clock::now();
#define DURATION std::cout << "Duration: " << double(std::chrono::duration_cast<std::chrono::microseconds>(End - Start).count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;

static int valid = 1;

void Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

void Pixels_Center_Extract(const yolo::BoxArray& objs, cv::Mat& img_in, cv::Point2f& center);

#endif