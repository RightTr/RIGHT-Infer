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
        k4a_pitch = config["K4a"]["pitch"] ? config["K4a"]["pitch"].as<float>() : 25.0;
        k4a2robot_x = config["K4a"]["k4a2robot_x"] ? config["K4a"]["k4a2robot_x"].as<float>() : -166.93;
        k4a2robot_y = config["K4a"]["k4a2robot_y"] ? config["K4a"]["k4a2robot_y"].as<float>() : -12.91;

        cout << "========== K4a Config Loaded ==========" << endl;
        cout << "[K4a]" << endl;
        cout << "  pitch         : " << k4a_pitch << endl;
        cout << "  k4a2robot_x   : " << k4a2robot_x << endl;
        cout << "  k4a2robot_y   : " << k4a2robot_y << endl;
        cout << "[Pcl_process]" << endl;
        cout << "  vg_leafsize   : " << vg_leafsize << endl;
        cout << "  sor_amount    : " << sor_amount << endl;
        cout << "  sor_dis       : " << sor_dis << endl;
        cout << "  ransac_dis    : " << ransac_dis << endl;
        cout << "  ransac_iters  : " << ransac_iters << endl;
        cout << "  basket_radius : " << basket_radius << endl;
        cout << "========================================" << endl;
    } 
    catch(const YAML::Exception& e) 
    {
        cerr << "YAML Error: " << e.what() << endl;
        return ;
    }
    return ;
}

void Realsense_Read_Parameters(string path)
{
    try 
    {
        YAML::Node config = YAML::LoadFile(path);

        cout << "========== Realsense Config Loaded ==========" << endl;
        cout << "[Realsense]" << endl;
        cout << "=============================================" << endl;
    } 
    catch(const YAML::Exception& e) 
    {
        cerr << "YAML Error: " << e.what() << endl;
        return ;
    }
    return ;
}