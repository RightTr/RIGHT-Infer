#ifndef PARAM_HPP
#define PARAM_HPP
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

using namespace std;

extern float vg_leafsize;
extern int sor_amount;
extern float sor_dis;
extern float ransac_dis;
extern int ransac_iters;
extern float k4a_pitch;
extern double basket_radius;
extern float k4a2robot_x;
extern float k4a2robot_y;

void K4a_Read_Parameters(string path);

#endif