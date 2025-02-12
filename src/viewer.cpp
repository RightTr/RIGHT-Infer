#include "pclprocess.hpp"

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::CloudViewer viewer("Basket Cloud Viewer");
    // std::string pcd_path = "/home/right/RIGHT-Infer/workspace/pcl/basket_cloud/basket2.ply";
    std::string pcd_path = "/home/right/RIGHT-Infer/workspace/pcl/output.ply";
    int index = 1;
    TIMESTART
    // while(index < 500)
    // {
    //     std::string pcd_path = "/home/right/RIGHT-Infer/datasets/ply/basket_dataset/basket" + std::to_string(index++) + ".ply";
    //     Input_PointCloud(pcd_path, pointcloud); 
    //     Vg_Filter(0.04, pointcloud); 
    //     Sor_Filter(50, 0.01, pointcloud);
    //     Ror_Filter(4, 0.1, pointcloud);
    //     Circle_Extract(pointcloud);
    //     std::cout << "Output Cloud Size:" << pointcloud->size() << std::endl;
    //     viewer.showCloud(pointcloud);
    //     usleep(10);
    // }
    TIMEEND
    DURATION
    Input_PointCloud(pcd_path, pointcloud); 
    viewer.showCloud(pointcloud);      
    while(1)
    {

    } 

    return 0;
}

