#include "sensor/realsense.hpp"

using namespace std;

void RealSense::Configuration_Default()
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_BGR8, 60);
    profile = pipe.start(cfg);

    COUT_GREEN_START
    std::cout << "Open Realsense Default Success!" << std::endl;
    COUT_COLOR_END

    rs2::video_stream_profile color_profile =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics_color = color_profile.get_intrinsics();

    intrin_color._fx = intrinsics_color.fx;
    intrin_color._fy = intrinsics_color.fy;
    intrin_color._cx = intrinsics_color.ppx;
    intrin_color._cy = intrinsics_color.ppy;
}

void RealSense::Configuration_RGBD()
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    profile = pipe.start(cfg);

    COUT_GREEN_START
    std::cout << "Open Realsense RGBD Success!" << std::endl;
    COUT_COLOR_END

    rs2::video_stream_profile color_profile =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics_color = color_profile.get_intrinsics();

    rs2::video_stream_profile depth_profile =
        profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics_depth = depth_profile.get_intrinsics();
    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();

    intrin_color._fx = intrinsics_color.fx;
    intrin_color._fy = intrinsics_color.fy;
    intrin_color._cx = intrinsics_color.ppx;
    intrin_color._cy = intrinsics_color.ppy;

    intrin_depth._fx = intrinsics_depth.fx;
    intrin_depth._fy = intrinsics_depth.fy;
    intrin_depth._cx = intrinsics_depth.ppx;
    intrin_depth._cy = intrinsics_depth.ppy;
    intrin_depth._depth_scale = depth_scale;
}

void RealSense::Configuration_Infrared_Only()
{
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 60);
    profile = pipe.start(cfg);
    rs2::video_stream_profile infrared_profile = 
        profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
    for(auto&& sensor : profile.get_device().query_sensors()) // Disable the infrared laser emitter of RealSense camera
    {
        if(sensor.supports(RS2_OPTION_EMITTER_ENABLED)) 
        {
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); 
        }
    }
    COUT_GREEN_START
    std::cout << "Open Realsense Infrared_Only Success!" << std::endl;
    COUT_COLOR_END
}

RealSense RealSense::Create_Default()
{
    RealSense rs;
    rs.Configuration_Default();
    return rs;
}

RealSense RealSense::Create_Infrared_Only()
{
    RealSense rs;
    rs.Configuration_Infrared_Only();
    return rs;
}

RealSense RealSense::Create_RGBD()
{
    RealSense rs;
    rs.Configuration_RGBD();
    return rs;
}

void RealSense::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{   
    rs2::align align_to_color(RS2_STREAM_COLOR);
    frameset = pipe.wait_for_frames();
    frameset = align_to_color.process(frameset);
    rs2::video_frame frame_color = frameset.get_color_frame();
    rs2::depth_frame frame_depth = frameset.get_depth_frame();
    image_rs_color = cv::Mat(frame_color.get_height(), frame_color.get_width(), CV_8UC3, (void*)frame_color.get_data());
    image_rs_depth = cv::Mat(frame_depth.get_height(), frame_depth.get_width(), CV_16UC1, (void*)frame_depth.get_data());
    image_rs_depth.convertTo(image_rs_depth, CV_8U, 255.0 / 1000);
    image_cv_color = image_rs_color;
    image_cv_depth = image_rs_depth;
}

void RealSense::Color_to_Cv(cv::Mat &image_cv_color)
{
    frameset = pipe.wait_for_frames();
    rs2::video_frame frame_color = frameset.get_color_frame();
    image_rs_color = cv::Mat(frame_color.get_height(), frame_color.get_width(), CV_8UC3, (void*)frame_color.get_data());
    image_cv_color = image_rs_color;
}

void RealSense::Infrared_to_Cv(cv::Mat &image_cv_infrared_left, cv::Mat &image_cv_infrared_right)
{
    frameset = pipe.wait_for_frames();
    rs2::video_frame frame_infrared_left = frameset.get_infrared_frame(1);
    rs2::video_frame frame_infrared_right = frameset.get_infrared_frame(2);
    image_rs_infrared_left = cv::Mat(frame_infrared_left.get_height(), frame_infrared_left.get_width(), 
                                CV_8UC1, (void*)frame_infrared_left.get_data());
    image_rs_infrared_right = cv::Mat(frame_infrared_right.get_height(), frame_infrared_right.get_width(), 
                                CV_8UC1, (void*)frame_infrared_right.get_data());
    cv::cvtColor(image_rs_infrared_left, image_cv_infrared_left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(image_rs_infrared_right, image_cv_infrared_right, cv::COLOR_GRAY2BGR);
}

void RealSense::Save_Image(int amount, std::string output_dir)
{
    if(amount <= frame_count)
    {
        return ; 
    }
    frameset = pipe.wait_for_frames();
    rs2::video_frame frame_infrared = frameset.get_infrared_frame(1);
    cv::Mat image_infrared_saved = cv::Mat(frame_infrared.get_height(), frame_infrared.get_width(), 
                                CV_8UC1, (void*)frame_infrared.get_data());
    image_infrared_saved.convertTo(image_infrared_saved, cv::COLOR_GRAY2BGR);
    string filename = output_dir + "basket_2nd_" + to_string(frame_count) + ".png";
    if(cv::imwrite(filename, image_infrared_saved))
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
    cv::imshow("Infrared Image", image_infrared_saved);
    cv::waitKey(10);
    usleep(50000);
}

