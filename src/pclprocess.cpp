#include "pclprocess.hpp"

void PclProcess::Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
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

void PclProcess::Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_ptr);
    // std::cout << "Vg PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PclProcess::Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    
    sor.setInputCloud(cloud_ptr);							
    sor.setMeanK(amount);										
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_ptr);	
    // std::cout << "Sor PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PclProcess::Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    
    ror.setInputCloud(cloud_ptr);        
    ror.setRadiusSearch(radius);       
    ror.setMinNeighborsInRadius(amount);  
    ror.filter(*cloud_ptr);     
    std::cout << "Ror PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PclProcess::Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{   
    if(cloud_ptr->size() < 200)
    {
        return ;
    }
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    std::vector<pcl::ModelCoefficients> coeff;
    ne.setInputCloud(cloud_ptr);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.5);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.15);
    seg.setRadiusLimits(0.2, 0.25);
    seg.setInputNormals(normals);
    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_ptr);

    coeff.push_back(*coefficients);
    std::cout << coeff.size() << std::endl;
    for (int i = 0; i < coeff.size(); i++)
    {
        
        std::cout << coeff.at(i).values[2] << std::endl;
    }
    
    
}


PclProcess::PclProcess()
{

}

PclProcess::~PclProcess()
{

  
}