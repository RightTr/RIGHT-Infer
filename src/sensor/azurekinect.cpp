#include "sensor/azurekinect.hpp"

using namespace std;

bool K4a::Open()
{
    try
    {
        device = k4a::device::open(K4A_DEVICE_DEFAULT);
        COUT_GREEN_START
        cout << "Open K4a Device Success!" << endl;
        COUT_COLOR_END
        return true;
    }
    catch(const std::exception& e)
    {
        COUT_RED_START
        cerr << "Open K4a Device Error!" << endl;
        COUT_COLOR_END
        return false;
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
    if(device.get_capture(&capture, chrono::milliseconds(1000)));
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

void K4a::Color_to_Cv(cv::Mat &image_cv_color)
{
    if(device.get_capture(&capture, chrono::milliseconds(1000)));
    {    
        image_k4a_color = capture.get_color_image();
        image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);
    }
}

void K4a::Depth_to_Cv(cv::Mat &image_cv_depth)
{
    if(device.get_capture(&capture, chrono::milliseconds(1000)));
    {    
        image_k4a_depth = capture.get_depth_image();
        image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
        image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
        image_cv_depth.convertTo(image_cv_depth, CV_8U);
    }
}

void K4a::Save_Image(int amount, std::string output_dir)
{   
    if(frame_count >= amount)
    {
        return ; 
    }
    if(device.get_capture(&capture, chrono::milliseconds(100)) && frame_count < amount)
    {
        image_k4a_color = capture.get_color_image();
        cv::Mat image_saved = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        string filename = output_dir + "obj_" + to_string(frame_count) + ".png";
        if(cv::imwrite(filename, image_saved))
        {
            COUT_YELLOW_START
            cout << "Save obj_" << frame_count << ".png Success!" << endl;
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
        usleep(5000);
    }
}