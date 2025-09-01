#include "sensor/realsense_infer.hpp"
#include "myinfer.hpp"

using namespace std;

void RealSense_Infer::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs)
{
    for (auto &obj : objs) 
    {
        if(obj.left >=0 && obj.right < image_cv_color.cols && obj.top >= 0 && obj.bottom <= image_cv_color.rows)
        {
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo::random_color(obj.class_label);
            cv::rectangle(image_cv_color, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                        cv::Scalar(b, g, r), 5);
            auto name = labels[obj.class_label];
            auto caption = cv::format("%s %.2f", name, obj.confidence);
            int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
            cv::rectangle(image_cv_color, cv::Point(obj.left - 3, obj.top - 33),
                        cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
            cv::putText(image_cv_color, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
            if (obj.seg) 
            {
                if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_color.cols && 
                    obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_color.rows)
                    {
                        cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                        mask.convertTo(mask, CV_8UC1);
                        cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                        cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR); 
                        cv::addWeighted(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 0.8, 0.0, mask);  
                        mask.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                    }
                }
        }
    }
}

void RealSense_Infer::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs)
{
    for (auto &obj : objs) 
    {
        if(obj.left >=0 && obj.right < image_cv_depth.cols && obj.top >= 0 && obj.bottom <= image_cv_depth.rows)
        {
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo::random_color(obj.class_label);
            cv::rectangle(image_cv_depth, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                        cv::Scalar(b, g, r), 5);
            auto name = labels[obj.class_label];
            auto caption = cv::format("%s %.2f", name, obj.confidence);
            int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
            cv::rectangle(image_cv_depth, cv::Point(obj.left - 3, obj.top - 33),
                        cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
            cv::putText(image_cv_depth, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16); 
            if (obj.seg) 
            {
                if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_depth.cols && 
                    obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_depth.rows)
                {
                    cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    mask.convertTo(mask, CV_8UC1);
                    cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                    cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 1.0, 0.0, mask);  
                    mask.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                }
            }
        }
    }
}

void RealSense_Infer::Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // cloud.clear();
    // rs2::depth_frame frame_depth = frameset.get_depth_frame(); 
    // for(int u = 0; u < frame_depth.get_width(); u+=10)
    // {
    //     for(int v = 0; v < frame_depth.get_height(); v+=10)
    //     {
    //         float depth_value = frame_depth.get_distance(u, v);
    //         if(depth_value != 0)
    //         {
    //             float x = (u - intrinsics_depth.ppx) * depth_value / intrinsics_depth.fx;
    //             float y = (v - intrinsics_depth.ppy) * depth_value / intrinsics_depth.fy;
    //             float z = depth_value;
    //             cloud.push_back(pcl::PointXYZ(x, y ,z));
    //         }
    //     }
    // } 
    // std::cout << "Global PointCloud:" << cloud.size() << std::endl;
}

void RealSense_Infer::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{

}
