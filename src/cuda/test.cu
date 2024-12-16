#include "../../inc/cutest.hpp"

__global__ void test()
{
    printf("CUDA!\n");
}


void CUDA_Test()
{
    
    test<<<1,6>>>();
    cudaDeviceSynchronize();

}



