#include "pclprocess.hpp"

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroidCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointXYZRGB centroidPoint;
    pcl::visualization::CloudViewer viewer("Basket Cloud Viewer");
    std::string pcd_path = "/home/right/RIGHT-Infer/workspace/pcl/output.ply";
    // Eigen::Vector4f centroid;

    PclProcess pclprocess;
    pclprocess.Input_PointCloud(pcd_path, pointcloud);
    // TIMESTART
    // pclprocess.Vg_Filter(0.01, pointcloud); 
    // pclprocess.Sor_Filter(70, 0.01, pointcloud);
    // pclprocess.Ror_Filter(200, 0.1, pointcloud);
    // TIMEEND
    // DURATION
    // pcl::compute3DCentroid(*pointcloud, centroid);

    
    // centroidPoint.x = centroid[0];
    // centroidPoint.y = centroid[1];
    // centroidPoint.z = centroid[2];
    // centroidPoint.r = 150;
    // centroidPoint.g = 200;
    // centroidPoint.b = 50;
    // centroidCloud->points.push_back(centroidPoint);

    // std::cout << "x:" << centroid.x() 
    //             << ",y:" << centroid.y() 
    //             << ",z:" << centroid.z() << std::endl;

    
    // viewer.showCloud(centroidCloud);
    viewer.showCloud(pointcloud);

    while(1) 
    {

    }

    return 0;
}

