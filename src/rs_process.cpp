#include "camera.hpp"
#include "myinfer.hpp"
#include "process_all_in_one.hpp"
#include "tcp_socket.hpp"

int main(int argc, char const *argv[])
{
    Yolo yolo;
    RealSense rs;
    std::string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket/best.engine";
    yolo::BoxArray objs;
    cv::Mat image_color;
    cv::Point2f center;
    string srv_ip = "127.0.0.1";
    uint16_t src_port = 8888;
    int img_width = 0;
    float pixel_diffx = 0;
    bool align_signal = 0;
    uint8_t buf[5] = {0};

    TcpSocket tcpsocket;
    CHECK_RET(tcpsocket.Socket());
    CHECK_RET(tcpsocket.Connect(srv_ip, src_port));

    while (1)
    {
        rs.Color_to_Cv(image_color);
        img_width = image_color.cols;
        yolo.Yolov8_Seg_Enable(engine_v8_seg);
        yolo.Single_Inference(image_color, objs);
        rs.Color_With_Mask(image_color, objs);
        Pixels_Center_Extract(objs, image_color, center);
        pixel_diffx = center.x - img_width / 2.;
        if(pixel_diffx < 5.)
        {
            align_signal = 1;
            cout << "Successfully Aligned!" << endl;
        }
        else
        {
            align_signal = 0;
            cout << "Try to Align......" << endl;
        }
        memcpy(buf, &align_signal, sizeof(align_signal));
        memcpy(buf + 1, &pixel_diffx, sizeof(pixel_diffx));
        cv::imshow("Seg Color Image", image_color);
        if (cv::waitKey(1) == 27)
            break;
        cout << "Client Send";
        tcpsocket.Send(buf, sizeof(buf));
        memset(buf, 0, sizeof(buf));
    }
    return 0;
}
