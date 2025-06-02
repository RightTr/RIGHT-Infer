#include "utils_all_in_one.hpp"

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
    std::cout << "Ror PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{   
    
    if(cloud_ptr->size() < 200)
    {
        return ;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    std::vector<pcl::ModelCoefficients> coeff;
    ne.setInputCloud(cloud_ptr);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.01);
    ne.compute(*normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setNormalDistanceWeight(0.001);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0.23, 0.24);
    seg.setInputNormals(normals);
    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_ptr);
    coeff.push_back(*coefficients); 
    std::cout << "x:"<< coeff.at(0).values[0] << ",y:" << coeff.at(0).values[1] << 
                ",z:" << coeff.at(0).values[2] << ",r:" << coeff.at(0).values[3] <<
                ",ex:"<< coeff.at(0).values[4] << ",ey:" << coeff.at(0).values[5]<< 
                ",ez" << coeff.at(0).values[6] << std::endl;  
    if(coeff.at(0).values[3] > 0.23 && coeff.at(0).values[3] < 0.24)
    {
        std::cout << "Valid Extract:" << valid++ << std::endl;
    }
}

void Pixels_Center_Extract(const yolo::BoxArray& objs, cv::Mat& img_in, cv::Point2f& center)
{
    for(auto& obj:objs)
    {
        int valid = 0;
        float x_sum = 0., y_sum = 0.;
        for(int i = obj.left; i <= obj.right; i++)
        {
            for(int j = obj.top; j <= obj.bottom; j++)
            {
                int index = j * img_in.cols + i;
                if (obj.seg->data[index] == 0)
                {
                    x_sum += i;
                    y_sum += j;
                    valid++;
                }
            }
        }
        if(valid > 0)
        {
            center = cv::Point2f(x_sum / valid, y_sum / valid);
            cv::circle(img_in, center, 3, cv::Scalar(0, 0, 255), -1);
            printf("Center: (%f, %f)\n", center.x, center.y);
        }
        else
        {
            return ;
        }
    }
}