#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/right/Infer/workspace/pcl/output_seg.ply", *pointcloud) == -1) 
    {
        PCL_ERROR("Couldn't Read File output.ply\n");
        return 0;
    }
    else
    {
        std::cout <<  pointcloud->size() << std::endl;
    }


    pcl::visualization::CloudViewer viewer("Basket Cloud Viewer");

    viewer.showCloud(pointcloud);

    while(1) 
    {

    }

    return 0;
}

