#include "sensor/realsense.hpp"
#include "sensor/camera_wrapper.hpp"
#include "featdetector/myinfer.hpp"


int main(int argc, char const *argv[])
{
    RealSense rs = RealSense::Create_Default();
    CameraWrapper<RealSense> rs_wrapper = CameraWrapper(rs);
    std::string plnet_config_path = "/home/right/RIGHT-Infer/config/feature_detector.yaml";
    std::string plnet_model_dir = "/home/right/RIGHT-Infer/models";
    YAML::Node file_node = YAML::LoadFile(plnet_config_path);
    PLNetConfig plnet_config;
    plnet_config.Load(file_node);
    plnet_config.SetModelPath(plnet_model_dir);

    auto feature_detector = std::shared_ptr<FeatureDetector>(new FeatureDetector(plnet_config));

    while(1)
    {
        cv::Mat image_cv_color;
        rs.Color_to_Cv(image_cv_color);
        cv::Mat image_cv_gray;
        Eigen::Matrix<float, 259, Eigen::Dynamic> features;
        std::vector<Eigen::Vector4d> lines;
        cv::cvtColor(image_cv_color, image_cv_gray, cv::COLOR_RGB2GRAY);
        feature_detector->Detect(image_cv_gray, features, lines);
        rs_wrapper.Draw_Lines(image_cv_gray, lines);

        cv::imshow("Screen", image_cv_gray);
        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}
