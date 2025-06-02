#include "params.hpp"

float vg_leafsize;
int sor_amount;
float sor_dis;
float ransac_dis;
int ransac_iters;
float k4a_pitch;
float k4a2robot_x;
float k4a2robot_y;
double basket_radius;

void K4a_Read_Parameters(string path)
{
    try 
    {
        YAML::Node config = YAML::LoadFile(path);
        vg_leafsize = config["Pcl_process"]["vg_leafsize"] ? config["Pcl_process"]["vg_leafsize"].as<float>() : 0.05;
        sor_amount = config["Pcl_process"]["sor_amount"] ? config["Pcl_process"]["sor_amount"].as<int>() : 50;
        sor_dis = config["Pcl_process"]["sor_dis"] ? config["Pcl_process"]["sor_dis"].as<float>() : 0.1;
        ransac_dis = config["Pcl_process"]["ransac_dis"] ? config["Pcl_process"]["ransac_dis"].as<float>() : 0.2;
        ransac_iters = config["Pcl_process"]["ransac_iters"] ? config["Pcl_process"]["ransac_iters"].as<float>() : 10000;
        basket_radius = config["Pcl_process"]["basket_radius"] ? config["Pcl_process"]["basket_radius"].as<double>() : 0.5;
        k4a_pitch = config["kinect"]["pitch"] ? config["kinect"]["pitch"].as<float>() : 25.0;
        k4a2robot_x = config["kinect"]["k4a2robot_x"] ? config["kinect"]["k4a2robot_x"].as<float>() : -166.93;
        k4a2robot_y = config["kinect"]["k4a2robot_y"] ? config["kinect"]["k4a2robot_y"].as<float>() : -12.91;
    } 
    catch(const YAML::Exception& e) 
    {
        std::cerr << "YAML Error: " << e.what() << std::endl;
        return ;
    }
    return ;
}