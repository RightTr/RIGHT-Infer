#ifndef PCLPROCESS_HPP
#define PCLPROCESS_HPP

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <chrono>

#define TIMESTART auto Start = std::chrono::system_clock::now();
#define TIMEEND auto End = std::chrono::system_clock::now();
#define DURATION std::cout << "Duration: " << double(std::chrono::duration_cast<std::chrono::microseconds>(End - Start).count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;



class PclProcess   
{
    private:

        pcl::VoxelGrid<pcl::PointXYZ> vg;   
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;	
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;


    public:
        void Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

        void Input_PointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in);

        void Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

        void Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

        void Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

        PclProcess();

        ~PclProcess();


};

#endif