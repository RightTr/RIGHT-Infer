#include "realsense.hpp"

int main(int argc, char const *argv[])
{
    RealSense rs = RealSense::Create_Infrared_Only();
    std::string output_dir = "/home/right/RIGHT-Infer/datasets/basket_ir_2nd/";
    while(1)
    {
        rs.Save_Image(800, output_dir);
    }

    return 0;
}