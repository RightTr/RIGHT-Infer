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
    color_intrinsics = k4aCalibration.color_camera_calibration;
}

void K4a::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{   
    if(device.get_capture(&capture, chrono::milliseconds(500)));
    {    
        image_k4a_color = capture.get_color_image();
        image_k4a_depth = capture.get_depth_image();
        image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
        image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);
        image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
        image_cv_depth.convertTo(image_cv_depth, CV_8U);
    }
}

void K4a::Save_Image(int amount)
{   
    if(device.get_capture(&capture, chrono::milliseconds(100)) && frame_count < amount)
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

void K4a::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray &objs)
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

void K4a::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray &objs)
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
                cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                mask.convertTo(mask, CV_8UC1);
                cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 1.0, 0.0, mask);  
                mask.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
            }
        }
    }
}

void K4a::Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    uint16_t* depth_data = (uint16_t*)image_k4a_depth_to_color.get_buffer();
    for (int v = 0; v < image_k4a_depth_to_color.get_height_pixels(); v+=9)
    {
        for (int u = 0; u < image_k4a_depth_to_color.get_width_pixels(); u+=9) 
        {   
            float depth_value = static_cast<float>(depth_data[v * image_k4a_depth_to_color.get_width_pixels() + u] / 1000.0);
            if(depth_value != 0)
            {
                float x = (u - color_intrinsics.intrinsics.parameters.param.cx) * depth_value / color_intrinsics.intrinsics.parameters.param.fx;
                float y = (v - color_intrinsics.intrinsics.parameters.param.cy) * depth_value / color_intrinsics.intrinsics.parameters.param.fy;
                float z = depth_value;
                cloud.push_back(pcl::PointXYZ(x, y ,z));
            }
        }
    }
    std::cout << "Global PointCloud:" << cloud.size() << std::endl;
}

void K4a::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs)
{
    cloud.clear();
    uint16_t* depth_data = (uint16_t*)image_k4a_depth_to_color.get_buffer();
    for(auto &obj : objs)
    {
        for (int v = obj.top; v < obj.top + 4 * obj.seg->height; v+=2)
        {
            for (int u = obj.left; u < obj.right; u+=2) 
            {     
                if(u < image_k4a_depth_to_color.get_width_pixels() && v < image_k4a_depth_to_color.get_height_pixels())
                {
                    float depth_value = static_cast<float>(depth_data[v * image_k4a_depth_to_color.get_width_pixels() + u] / 1000.0);
                    if(depth_value != 0)
                    {
                        float x = (u - color_intrinsics.intrinsics.parameters.param.cx) * depth_value / color_intrinsics.intrinsics.parameters.param.fx;
                        float y = (v - color_intrinsics.intrinsics.parameters.param.cy) * depth_value / color_intrinsics.intrinsics.parameters.param.fy;
                        float z = depth_value;
                        cloud.push_back(pcl::PointXYZ(x, y ,z));
                    }
                }
            }
        }
    }
    std::cout << "Mask PointCloud:" << cloud.size() << std::endl;
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
    depth_profile = frame_depth.get_profile().as<rs2::video_stream_profile>(); 
    intrinsics_depth = depth_profile.get_intrinsics();
    pointcloud_rs.map_to(frame_color);
    points = pointcloud_rs.calculate(frame_depth);
    // std::cout << intrinsics_depth.ppy << std::endl;
    // float depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
    // float depth_offset = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_option(RS2_OPTION_DEPTH_UNITS);
    // std::cout << depth_offset << std::endl;
    image_rs_color = cv::Mat(frame_color.get_height(), frame_color.get_width(), CV_8UC3, (void*)frame_color.get_data());
    image_rs_depth = cv::Mat(frame_depth.get_height(), frame_depth.get_width(), CV_16UC1, (void*)frame_depth.get_data());
    image_rs_depth.convertTo(image_rs_depth, CV_8U, 255.0 / 1000);
    image_cv_color = image_rs_color;
    image_cv_depth = image_rs_depth;
}

void RealSense::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray objs)
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
                    mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    mask.convertTo(mask, CV_8UC1);
                    cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR); 
                    cv::addWeighted(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 0.8, 0.0, mask);  
                    mask.copyTo(image_cv_color(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
            }
        }
    }
}

void RealSense::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray objs)
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
                if(obj.left >= 0 && obj.seg->width >=0 && obj.left + obj.seg->width < image_cv_depth.cols && obj.top >= 0 && obj.seg->height >= 0 && obj.top + obj.seg->height <= image_cv_depth.rows)
                {
                    mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
                    mask.convertTo(mask, CV_8UC1);
                    cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
                    cv::addWeighted(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 1.0, 0.0, mask);  
                    mask.copyTo(image_cv_depth(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
                }
            }
        }
    }
}

void RealSense::Rs_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    pcl::PointXYZ point;
    const rs2::vertex* vertices = points.get_vertices();
    for(int i = 0; i < points.size(); i+=13)
    {
        point.x = vertices[i].x;
        point.y = vertices[i].y;
        point.z = vertices[i].z;
        cloud.push_back(point);
    }
    std::cout << "Global PointCloud:" << cloud.size() << std::endl;
    pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output.ply", cloud);
}

void RealSense::Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    cloud.clear();
    rs2::depth_frame frame_depth = frameset.get_depth_frame(); 
    for(int u = 0; u < frame_depth.get_width(); u+=14)
    {
        for(int v = 0; v < frame_depth.get_height(); v+=14)
        {
            float depth_value = frame_depth.get_distance(u, v);
            if(depth_value != 0)
            {
                float x = (u - intrinsics_depth.ppx) * depth_value / intrinsics_depth.fx;
                float y = (v - intrinsics_depth.ppy) * depth_value / intrinsics_depth.fy;
                float z = depth_value;
                cloud.push_back(pcl::PointXYZ(x, y ,z));
            }
        }
    } 
    std::cout << "Global PointCloud:" << cloud.size() << std::endl;
    pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/workspace/pcl/output.ply", cloud);
}

void RealSense::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud)
{

}

RealSense::RealSense()
{
    Configuration();
}

RealSense::~RealSense()
{
    
}