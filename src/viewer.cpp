#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "pclprocess.hpp"

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string pcd_path = "/home/right/RIGHT-Infer/workspace/pcl/output.ply";
    PclProcess pclprocess;
    pclprocess.Input_PointCloud(pcd_path, pointcloud);
    TIMESTART
    // pclprocess.Vg_Filter(0.01, pointcloud);
    
    // pclprocess.Sor_Filter(70, 0.01, pointcloud);
    

    // pclprocess.Ror_Filter(200, 0.1, pointcloud);
   
    TIMEEND
    DURATION



    pcl::visualization::CloudViewer viewer("Basket Cloud Viewer");

    viewer.showCloud(pointcloud);

    while(1) 
    {

    }

    return 0;
}

