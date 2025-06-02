#include "myinfer.hpp"

yolo::Image Yolo::cvimg(const cv::Mat &image) 
{ 
  return yolo::Image(image.data, image.cols, image.rows); 
}

void Yolo::Yolov8_Enable(std::string &engine_)
{
  type = yolo::Type::V8;
  engine = engine_;
}

void Yolo::Yolov8_Seg_Enable(std::string &engine_seg)
{
  type = yolo::Type::V8Seg;
  engine = engine_seg;
}

void Yolo::Single_Inference(std::string path)
{
  cv::Mat image = cv::imread(path);
  auto yolo = yolo::load(engine, type);
  if (yolo == nullptr) return;

  auto objs = yolo->forward(cvimg(image));
  int i = 0;
  for (auto &obj : objs) {
    uint8_t b, g, r;
    std::tie(b, g, r) = yolo::random_color(obj.class_label);
    cv::rectangle(image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                  cv::Scalar(b, g, r), 5);

    auto name = labels[obj.class_label];
    auto caption = cv::format("%s %.2f", name, obj.confidence);
    int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
    cv::rectangle(image, cv::Point(obj.left - 3, obj.top - 33),
                  cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
    cv::putText(image, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);

    if (obj.seg) {
      cv::imwrite(cv::format("%d_mask.jpg", i),
                  cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data));
      i++;
    }
  }

  printf("Save result to Result.jpg, %d objects\n", (int)objs.size());
  cv::imwrite(path + "result.jpg", image);
}

void Yolo::Single_Inference(cv::Mat &image)
{
  auto yolo = yolo::load(engine, type);
  if (yolo == nullptr) return;

  auto objs = yolo->forward(cvimg(image));
  for (auto &obj : objs) {
    uint8_t b, g, r;
    std::tie(b, g, r) = yolo::random_color(obj.class_label);
    cv::rectangle(image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                  cv::Scalar(b, g, r), 5);

    auto name = labels[obj.class_label];
    auto caption = cv::format("%s %.2f", name, obj.confidence);
    int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
    cv::rectangle(image, cv::Point(obj.left - 3, obj.top - 33),
                  cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
    cv::putText(image, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);

    if (obj.seg) 
    {

      cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
      cv::Mat mask(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
      mask.convertTo(mask, CV_8UC1);
      cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR); 
      cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR); 
      cv::Mat result;
      cv::addWeighted(image(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)), 1.0, mask, 0.8, 0.0, mask);  

      imshow("Mask", mask);
      mask.copyTo(image(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
     

    }
  }
}

void Yolo::Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out)
{
  if(!load_flag)
  {
    yolo = yolo::load(engine, type);
    load_flag = 1;
  }
  if (yolo == nullptr) return;

  auto Start = std::chrono::system_clock::now();
  
  auto objs = yolo->forward(cvimg(image));

  auto End = std::chrono::system_clock::now();
  auto Duration = std::chrono::duration_cast<std::chrono::microseconds>(End - Start);
  std::cout << "Infer Duration: " << double(Duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;
  objs_out = objs;
}

Yolo::Yolo()
{
  load_flag = 0;
}

Yolo::~Yolo()
{

}
