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
    // std::cout << "Ror PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PclProcess::Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr circle3d(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud_ptr));
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle3d);
    std::vector<int> ransac_inliers; 
    Eigen::VectorXf coeff;
    ransac.setDistanceThreshold(0.03);							
	ransac.setMaxIterations(10000);								
	ransac.computeModel();
	ransac.getModelCoefficients(coeff);	
    ransac.getInliers(ransac_inliers);	
    inliers->indices = ransac_inliers;
    extract.setInputCloud(cloud_ptr); 
    extract.setIndices(inliers); 
    extract.setNegative(false); 
    extract.filter(*cloud_ptr);					                            
    std::cout << cloud_ptr->size() << std::endl;
    std::cout << "x:" << coeff[0] << ",y:" << coeff[1] << ",z:" << coeff[2] << ",r=" << coeff[3] 
	     	<< ",ex:" << coeff[4] << ",ey:" << coeff[5] << ",ez:" << coeff[6] << std::endl;
}


PclProcess::PclProcess()
{

}

PclProcess::~PclProcess()
{

  
}