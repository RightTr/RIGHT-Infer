#include "realsense.hpp"
#include "myinfer.hpp"
#include "utils_all_in_one.hpp"
#include "tcp_socket.hpp"
#include "params.hpp"

#define PI 3.1415926535

int main(int argc, char const *argv[])
{
    Yolo yolo;
    RealSense rs = RealSense::Create_Default();
    string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket/best.engine";
    yolo::BoxArray objs;
    cv::Mat image_color;
    vector<float> center(2);
    string srv_ip = "127.0.0.1";
    uint16_t src_port = 8888;
    int img_width = 0;
    float angle_diffx = 0;
    bool align_signal = 0;
    uint8_t buf[5] = {0};
    FPSCounter fps("Realsense Stream");

    cv::Mat image_infrared_left, image_infrared_right;

    TcpSocket tcpsocket;
    if (!tcpsocket.Socket())
        throw runtime_error("Socket failed");
    if (!tcpsocket.Connect(srv_ip, src_port))
        throw runtime_error("Connect failed");

    float fx = rs.intrinsics_color.fx;
    float cx = rs.intrinsics_color.ppx;

    while (1)
    {
        rs.Color_to_Cv(image_color);
        img_width = image_color.cols;
        yolo.Yolov8_Seg_Enable(engine_v8_seg);
        cout << image_infrared_left.channels() << endl;
        yolo.Single_Inference(image_infrared_left, objs);
        rs.Color_With_Mask(image_infrared_left, objs);
        Pixels_Center_Extract(objs, image_color, center);
        angle_diffx = atan2f((center[0] - cx), fx) * 180 / PI;
        if(angle_diffx < 5. && angle_diffx > -5.)
        {
            align_signal = 1;
            COUT_GREEN_START
            cout << "Successfully Aligned!" << endl;
            COUT_COLOR_END
        }
        else
        {
            align_signal = 0;
            COUT_YELLOW_START
            cout << "Try to Align......" << endl;
            COUT_COLOR_END
        }
        memcpy(buf, &align_signal, sizeof(align_signal));
        memcpy(buf + 1, &angle_diffx, sizeof(angle_diffx));
        cv::imshow("Seg Color Image", image_color);
        if (cv::waitKey(1) == 27)
            break;

        cout << "Client Send\n";
        tcpsocket.Send(buf);
        fps.tick();
        memset(buf, 0, sizeof(buf));
    }
    return 0;
}


