#include "camera.hpp"
#include "myinfer.hpp"

using namespace std;

void K4a::Open()
{
    device = k4a::device::open(K4A_DEVICE_DEFAULT);
    if(!device)
    {
        COUT_RED_START
        cerr << "Open K4a Device Error!" << endl;
        COUT_COLOR_END
    }
    else
    {   
        COUT_GREEN_START
        cout << "Open K4a Device Success!" << endl;
        COUT_COLOR_END
    }
} 

void K4a::Installed_Count()
{
    device_count = k4a::device::get_installed_count();
    if(device_count == 0)
    {
        COUT_RED_START
        cout << "No K4a Device Found!" << endl;
        COUT_COLOR_END
    }
    else
    {
        COUT_BLUE_START
        cout << "Find " << device_count << " Device(s)" << endl;
        COUT_COLOR_END
    }
}

void K4a::Configuration()
{
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    device.start_cameras(&config);

    COUT_GREEN_START
    cout << "Start Device Success!" << endl;
    COUT_COLOR_END

    k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);
    k4aTransformation = k4a::transformation(k4aCalibration);
}

void K4a::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{   
    if(device.get_capture(&capture, chrono::milliseconds(500)));
    {    
        image_k4a_color = capture.get_color_image();
        image_k4a_depth = capture.get_depth_image();
        image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
        image_k4a_depth_to_pcl = k4aTransformation.depth_image_to_point_cloud(image_k4a_depth, K4A_CALIBRATION_TYPE_DEPTH);
        

        image_cv_xyz = cv::Mat(image_k4a_depth_to_pcl.get_height_pixels(), image_k4a_depth_to_pcl.get_width_pixels(), CV_16SC3, 
                                (void *)image_k4a_depth_to_pcl.get_buffer(), static_cast<size_t>(image_k4a_depth_to_pcl.get_stride_bytes()));

        
        
        image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);

        image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
        image_cv_depth.convertTo(image_cv_depth, CV_8U);
        cv::resize(image_cv_xyz, image_cv_xyz, image_cv_depth.size(), 0, 0, cv::INTER_LINEAR);
        // cv::imshow("xyz", image_cv_xyz);
    }
}

void K4a::Save_Image(int amount)
{   

    if(device.get_capture(&capture, chrono::milliseconds(500)) && frame_count < amount)
    {
        image_k4a_color = capture.get_color_image();
        cv::Mat image_saved = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());

        filename = output_dir + "basket_" + to_string(frame_count) + ".png";

        if(cv::imwrite(filename, image_saved))
        {
            COUT_YELLOW_START
            cout << "Save basket_" << frame_count << ".png Success!" << endl;
            COUT_COLOR_END
            frame_count++;
        }
        else
        {
            COUT_RED_START
            cout << "Save error!" << endl;
            COUT_COLOR_END
        }
        image_saved.release();
        usleep(500000);
    }


}

void K4a::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs)
{
    for (auto &obj : objs) 
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
            if(obj.left >=0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_color.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_color.rows)
            {
                mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);

                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 


                cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR); 

                cv::addWeighted(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask_color, 0.8, 0.0, mask_color);  

                mask_color.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
            }

        }
    }

}

void K4a::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs)
{
    for (auto &obj : objs) 
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
            if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_depth.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_depth.rows)
            {
                mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);
                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                mask_depth = mask;
                cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask_depth, 1.0, 0.0, mask_depth);  
                mask_depth.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
            }
        }
    }
}

void K4a::Mask_to_Binary(cv::Mat &image_cv_binary, yolo::BoxArray objs)
{
    image_mask_binary = cv::Mat::zeros(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC1);
    
    for (auto &obj : objs)
    {
        if (obj.seg) 
        {
            if(obj.left >=0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_mask_binary.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_mask_binary.rows)
            {
                mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);

                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);  

                mask.copyTo(image_mask_binary(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                // cv::imshow("Binary", image_mask_binary);
            }

        }
    }
    image_cv_binary = image_mask_binary;
}

void K4a::Mask_to_Binary(yolo::BoxArray objs)
{
    image_mask_binary = cv::Mat::zeros(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC1);
    
    for (auto &obj : objs)
    {
        if (obj.seg) 
        {
            if(obj.left >=0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_mask_binary.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_mask_binary.rows)
            {
                mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);

                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);  

                mask.copyTo(image_mask_binary(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                // cv::imshow("Binary", image_mask_binary);
            }

        }
    }
}

void K4a::K4a_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    // cloud.clear();
    // for (int v = 0; v < image_k4a_depth_to_color.get_height_pixels(); v+=4)
    // {
    //     for (int u = 0; u < image_k4a_depth_to_color.get_width_pixels(); u+=4) 
    //     {   
    //         int valid = 0;
    //         k4a_float2_t point2d{static_cast<float>(u), static_cast<float>(v)};
    //         k4a_float3_t point3d ={0};
    //         pcl::PointXYZ point;

    //         uint16_t depth = image_k4a_depth_to_color.get_buffer()[v * image_k4a_depth_to_color.get_width_pixels() + u];

    //         if (depth == 0) continue;

    //         k4a_calibration_2d_to_3d(&calibration_depth, &point2d, depth / 1000.0f, 
    //                                 K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH,
    //                                 &point3d, &valid);

    //         if(valid)
    //         {
    //             point.x = point3d.xyz.x;
    //             point.y = point3d.xyz.y;
    //             point.z = point3d.xyz.z;
    //             cloud.push_back(point);
    //         }
    //     }
    // }

    // std::cout << "PointCloud:" << cloud.size() << std::endl;
    // pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output.ply", cloud);

}

void K4a::Cv_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{      
    cloud.clear();
    
    pcl::PointXYZ point;
    for (int v = 0; v < image_cv_xyz.rows; v+=8)
    {
        for (int u = 0; u < image_cv_xyz.cols; u+=8) 
        {   
            
            cv::Vec3s point3d = image_cv_xyz.at<cv::Vec3s>(v, u);
            point.x = static_cast<float>(point3d[1]) / 1000.0f;
            point.y = static_cast<float>(point3d[2]) / 1000.0f; 
            point.z = static_cast<float>(point3d[0]) / 1000.0f; 
            cloud.push_back(point);
        }
    }

    std::cout << "Global PointCloud:" << cloud.size() << std::endl;
    pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output.ply", cloud);

}

void K4a::K4a_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // cloud.clear();
    // std::vector<cv::Point> nonzeros;
    // if(mask_depth.empty())
    // {   
    //     std::cout << "Mask is Empty!" << std::endl;
    //     return ;
    // }
    // else
    // {
    //     cv::findNonZero(mask_depth, nonzeros); 
    //     std::cout << "NonZeroPoints:" << nonzeros.size() << std::endl;
    // }

    // for(int i = 0; i < nonzeros.size(); i++)
    // {

    //     int valid = 0;
    //     k4a_float2_t point2d{static_cast<float>(nonzeros[i].x), static_cast<float>(nonzeros[i].y)};
    //     k4a_float3_t point3d ={0};
    //     pcl::PointXYZ point;

    //     uint16_t depth = image_k4a_depth.get_buffer()[nonzeros[i].y * image_k4a_depth.get_width_pixels() + nonzeros[i].x];

    //     if (depth == 0) continue;

    //     k4a_calibration_2d_to_3d(&calibration_depth, &point2d, depth / 1000.0f, 
    //                             K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH,
    //                             &point3d, &valid);

    //     if(valid)
    //     {
    //         point.x = point3d.xyz.x;
    //         point.y = point3d.xyz.y;
    //         point.z = point3d.xyz.z;
    //         cloud.push_back(point);
    //     }
    // }
    // std::cout << "Seg PointCloud:" << cloud.size() << std::endl;
    // pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output.ply", cloud);


}

void K4a::Cv_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    std::vector<cv::Point> nonzeros;
    pcl::PointXYZ point;
    if(image_mask_binary.empty())
    {   
        std::cout << "Binary Mask is Empty!" << std::endl;
        return ;
    }
    else
    {
        cv::findNonZero(image_mask_binary, nonzeros); 
        // std::cout << "NonZeroPoints:" << nonzeros.size() << std::endl;
    }

    // for (int v = 0; v < image_mask_binary.rows; v+=2)
    // {
    //     for (int u = 0; u < image_mask_binary.cols; u+=2)
    //     {   
    //         if(image_mask_binary.at<uchar>(v, u) != 0)
    //         {
    //             cv::Vec3s point3d = image_cv_xyz.at<cv::Vec3s>(v, u);
    //             point.x = static_cast<float>(point3d[0]) / 1000.0f;
    //             point.y = static_cast<float>(point3d[1]) / 1000.0f; 
    //             point.z = static_cast<float>(point3d[2]) / 1000.0f; 
    //             cloud.push_back(point);
    //         }
    //     }
    // }
    for(int i = 0; i < nonzeros.size(); i++)
    {
        cv::Vec3s point3d = image_cv_xyz.at<cv::Vec3s>(nonzeros[i].y, nonzeros[i].x);

        point.x = static_cast<float>(point3d[1]) / 1000.0f;
        point.y = static_cast<float>(point3d[2]) / 1000.0f; 
        point.z = static_cast<float>(point3d[0]) / 1000.0f; 
        if(sqrt(point.x * point.x + point.y * point.y + point.z * point.z) < MIN_DISTANCE) continue ;
        cloud.push_back(point);
    }
    std::cout << "Seg PointCloud:" << cloud.size() << std::endl;
    pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output_seg.ply", cloud);

}

K4a::K4a()
{
    Installed_Count();
    Open();
    Configuration();
}

K4a::~K4a()
{
    image_k4a_depth.reset();
    image_k4a_color.reset();
    capture.reset();
    device.close();
}


void RealSense::Configuration()
{
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    profile = pipe.start(cfg);
    COUT_GREEN_START
    std::cout << "Open Realsense Device Success!" << std::endl;
    COUT_COLOR_END
}

void RealSense::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{   
    rs2::align align_to_color(RS2_STREAM_COLOR);
    frameset  = pipe.wait_for_frames();
    frameset = align_to_color.process(frameset);
    rs2::video_frame frame_color = frameset.get_color_frame();
    rs2::depth_frame frame_depth = frameset.get_depth_frame();
    // depth_profile = frame_depth_ptr->get_profile().as<rs2::video_stream_profile>(); 
    // intrinsics = depth_profile.get_intrinsics();
    // depth_value = depth_frame.get_distance(x, y);
    image_cv_color = cv::Mat(frame_color.get_height(), frame_color.get_width(), CV_8UC3, (void*)frame_color.get_data());
    image_cv_depth = cv::Mat(frame_depth.get_height(), frame_depth.get_width(), CV_16UC1, (void*)frame_depth.get_data());
    image_cv_depth.convertTo(image_cv_depth, CV_8U, 255.0 / 1000);

}

void RealSense::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs)
{
    for (auto &obj : objs) 
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
            if(obj.left >=0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_color.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_color.rows)
            {
                mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);
                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR); 
                cv::addWeighted(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask_color, 0.8, 0.0, mask_color);  
                mask_color.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
            }
        }
    }
}

void RealSense::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs)
{
    for (auto &obj : objs) 
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
            if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_depth.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_depth.rows)
            {
                mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);
                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                mask_depth = mask;
                cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask_depth, 1.0, 0.0, mask_depth);  
                mask_depth.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
            }
        }
    }
}
RealSense::RealSense()
{
    Configuration();
}

RealSense::~RealSense()
{
    
}