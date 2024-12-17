#include "pclprocess.hpp"

void PCLPROCESS::Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(pcd_path, *cloud_ptr) == -1) 
    {
        PCL_ERROR("Couldn't Read File\n");
        return ;
    }
    else
    {
        std::cout << "Input Cloud Size:" <<  cloud_ptr->size() << std::endl;
    }
}

void PCLPROCESS::Input_PointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in)
{
    // cloud_ptr = &cloud_in;
}

void PCLPROCESS::Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_ptr);
    std::cout << "Vg PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PCLPROCESS::Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;	
    sor.setInputCloud(cloud_ptr);							
    sor.setMeanK(amount);										
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_ptr);	
    std::cout << "Sor PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PCLPROCESS::Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_ptr);        
    ror.setRadiusSearch(radius);       
    ror.setMinNeighborsInRadius(amount);  
    ror.filter(*cloud_ptr);     
    std::cout << "Ror PointCloud Size:" << cloud_ptr->size() << std::endl;
}

PCLPROCESS::PCLPROCESS()
{


}

PCLPROCESS::~PCLPROCESS()
{

  
}