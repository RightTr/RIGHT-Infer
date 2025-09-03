#include <iostream>

#define COUT_RED_START      std::cout << "\033[1;31m";
#define COUT_GREEN_START    std::cout << "\033[1;32m";
#define COUT_YELLOW_START   std::cout << "\033[1;33m";
#define COUT_BLUE_START     std::cout << "\033[1;34m";
#define COUT_PURPLE_START   std::cout << "\033[1;35m";
#define COUT_CYAN_START     std::cout << "\033[1;36m";
#define COUT_WHITE_START    std::cout << "\033[1;37m";
#define COUT_COLOR_END      std::cout << "\033[0m";

struct ColorIntrinsics
{
    float _fx;
    float _fy;
    float _cx;
    float _cy;
    float _depth_scale;

    ColorIntrinsics() :
        _fx(0), _fy(0), _cx(0), _cy(0) {}

    ColorIntrinsics(float fx, float fy, float cx, float cy) :
        _fx(fx), _fy(fy), _cx(cx), _cy(cy) {}

};

struct DepthIntrinsics
{
    float _fx;
    float _fy;
    float _cx;
    float _cy;
    float _depth_scale;

    DepthIntrinsics() :
        _fx(0), _fy(0), _cx(0), _cy(0), _depth_scale(0.001f) {}

    DepthIntrinsics(float fx, float fy, float cx, float cy) :
        _fx(fx), _fy(fy), _cx(cx), _cy(cy), _depth_scale(0.001f) {}

    DepthIntrinsics(float fx, float fy, float cx, float cy, float depth_scale) :
        _fx(fx), _fy(fy), _cx(cx), _cy(cy), _depth_scale(depth_scale) {}
};