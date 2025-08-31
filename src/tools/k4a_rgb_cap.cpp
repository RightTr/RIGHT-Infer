#include "azurekinect.hpp"

int main(int argc, char const *argv[])
{
    K4a k4a;
    std::string output_dir = "/home/right/Datasets/";
    while(1)
    {
        k4a.Save_Image(800, output_dir);
    }

    return 0;
}