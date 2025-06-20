#include "realsense.hpp"

int main(int argc, char const *argv[])
{
    RealSense rs = RealSense::Create_Infrared_Only();

    while(1)
    {
        rs.Save_Image(800);
    }

    return 0;
}