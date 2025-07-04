#include "azurekinect.hpp"

int main(int argc, char const *argv[])
{
    K4a k4a;

    while(1)
    {
        k4a.Save_Image(800);
    }

    return 0;
}