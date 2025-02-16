#include "pclprocess.hpp"

void Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
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

void Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{   
    pcl::VoxelGrid<pcl::PointXYZ> vg;   
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_ptr);
    // std::cout << "Vg PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;	
    sor.setInputCloud(cloud_ptr);							
    sor.setMeanK(amount);										
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_ptr);	
    // std::cout << "Sor PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_ptr);        
    ror.setRadiusSearch(radius);       
    ror.setMinNeighborsInRadius(amount);  
    ror.filter(*cloud_ptr);     
    // std::cout << "Ror PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::VectorXf &coeff)
{   
    if(cloud_ptr->size() < 50)
    {
        return ;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr circle3d(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud_ptr));
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle3d);
    std::vector<int> ransac_inliers; 
    ransac.setDistanceThreshold(0.04);							
	ransac.setMaxIterations(1000);								
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
    // if(coeff[3] < 0.24 && coeff[3] > 0.19)
    // {
    //     valid++;
    //     std::cout << "Valid Extract:" << valid << std::endl;
    // }
}
