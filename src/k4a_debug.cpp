#include "sensor/azurekinect_infer.hpp"
#include "featdetector/myinfer.hpp"


int main(int argc, char const *argv[])
{
    K4a_Infer k4a_infer;
    std::string plnet_config_path = "/home/right/RIGHT-Infer/config/feature_detector.yaml";
    std::string plnet_model_dir = "/home/right/RIGHT-Infer/models";
    YAML::Node file_node = YAML::LoadFile(plnet_config_path);
    PLNetConfig plnet_config;
    plnet_config.Load(file_node);
    plnet_config.SetModelPath(plnet_model_dir);

    auto feature_detector = std::shared_ptr<FeatureDetector>(new FeatureDetector(plnet_config));
    
    while(1)
    {   
        cv::Mat image_color, image_gray;
        Eigen::Matrix<float, 259, Eigen::Dynamic> features;
        std::vector<Eigen::Vector4d> lines;
        k4a_infer.Color_to_Cv(image_color);
        cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);
        feature_detector->Detect(image_gray, features, lines);
        k4a_infer.Draw_Lines(image_gray, lines);


        cv::imshow("Color Image", image_gray);
        if (cv::waitKey(1) == 27) break;

    }
    return 0;
}