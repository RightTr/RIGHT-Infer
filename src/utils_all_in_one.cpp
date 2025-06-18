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

void Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::Vector2f& target2d)
{   
    if (cloud_ptr->size() < 20)
    {
        cerr << "No enough points when extracting circle" << endl;
        target2d = Eigen::Vector2f(-999.0f, -999.0f); // as NAN
        return;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr circle3d(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud_ptr));
    Eigen::VectorXf coeff;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle3d);
    vector<int> ransac_inliers;
    ransac.setDistanceThreshold(ransac_dis);
    ransac.setMaxIterations(ransac_iters);
    ransac.computeModel();
    ransac.getModelCoefficients(coeff);
    ransac.getInliers(ransac_inliers);
    inliers->indices = ransac_inliers;
    if (inliers->indices.empty())
    {
        cerr << "Get no point in circle extracted" << endl;
        target2d = Eigen::Vector2f(-999.0f, -999.0f); // as NAN
        return;
    }
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_ptr);

    Eigen::Vector3d center3d = FitCircle_LM(cloud_ptr, basket_radius, coeff);

    double radians = k4a_pitch * M_PI / 180.0;

    float target2robot_x= center3d[0] * 1000 + k4a2robot_x;
    float target2robot_y = center3d[1] * sin(radians) * 1000 + center3d[2] * cos(radians) * 1000 + k4a2robot_y;

    target2d = Eigen::Vector2f(target2robot_x, target2robot_y);

    cout << "target2robot_x: " << target2robot_x << ", target2robot_y: " << target2robot_y << endl;
}

Eigen::Vector3d FitCircle_LM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff)
{
    Eigen::VectorXd c(3);
    c << coeff[0], coeff[1], coeff[2];

    CircleFunctor functor(cloud_ptr, radius);
    Eigen::LevenbergMarquardt<CircleFunctor> lm(functor);
    lm.minimize(c);

    return Eigen::Vector3d(c(0), c(1), c(2));
}

void Pixels_Center_Extract(const yolo::BoxArray& objs, cv::Mat& img_in, vector<float>& target2d)
{
    for(auto& obj:objs)
    {
        target2d.clear();
        float valid = 0;
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
            target2d.push_back(x_sum / valid);
            target2d.push_back(y_sum / valid);
            cv::circle(img_in, cv::Point2f(target2d[0], target2d[1]), 3, cv::Scalar(0, 0, 255), -1);
            printf("Center: (%f, %f)\n", target2d[0], target2d[1]);
        }
        else
        {
            return ;
        }
    }
}

