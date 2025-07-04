#include "azurekinect.hpp"
#include "myinfer.hpp"
#include "utils_all_in_one.hpp"
#include "mythread.hpp"

int main(int argc, char const *argv[])
{
    K4a_Read_Parameters("/home/right/RIGHT-Infer/config/kinect.yaml");

    Mythread mythread;

    pthread_t thread[8] = {0};

    pthread_create(&thread[0], NULL, Mythread::K4a_Single_Inference_V8_Seg, &mythread);
    pthread_create(&thread[1], NULL, Mythread::K4a_Image_Show, &mythread);
    pthread_create(&thread[2], NULL, Mythread::TCP_Server, &mythread);
    pthread_create(&thread[3], NULL, Mythread::Pcl_Process, &mythread);

    for(int i = 0; i < 8; i++)
    {
        pthread_join(thread[i], NULL);
    }

    return 0;
}



