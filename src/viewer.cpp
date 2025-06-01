#include "process_all_in_one.hpp"

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::CloudViewer viewer("Basket Cloud Viewer");
    // std::string pcd_path = "/home/right/RIGHT-Infer/workspace/pcl/basket_cloud/basket2.ply";
    // std::string pcd_path = "/home/right/RIGHT-Infer/workspace/pcl/output.ply";
    int index = 1;
    TIMESTART
    while(index < 315)
    {
        std::string pcd_path = "/home/right/RIGHT-Infer/datasets/ply/test/basket" + std::to_string(index++) + ".ply";
        Input_PointCloud(pcd_path, pointcloud);
        
        Vg_Filter(0.06, pointcloud); 
        Sor_Filter(50, 0.01, pointcloud);
        // Ror_Filter(5, 0.1, pointcloud);
        Circle_Extract(pointcloud);
        

        std::cout << "Output Cloud Size:" << pointcloud->size() << std::endl;
        viewer.showCloud(pointcloud);



    }
    TIMEEND
    DURATION
    return 0;
}

