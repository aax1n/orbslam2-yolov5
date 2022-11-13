#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>
using namespace std;

class YoloDetection
{
public:
    YoloDetection();
    ~YoloDetection();
    void GetImage(cv::Mat& RGB);
    void ClearImage();
    bool Detect();
    void ClearArea();
    vector<cv::Rect2i> mvPersonArea = {};
    vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5);

public:
    cv::Mat mRGB;
    torch::jit::script::Module mModule;
    std::vector<std::string> mClassnames;

    // 6-28
    vector<string> mvDynamicNames;
    vector<string> mvStaicNames;
    vector<cv::Rect2i> mvDynamicArea;
    vector<cv::Rect2i> mvDetectArea;
    /// view 用
    map<string, vector<cv::Rect2i>> mmDetectMap;
    map<string, map<float, vector<cv::Rect2i>>> _mmDetectMap;
    map<string, vector<cv::Rect2i>> cp_mmDetectMap;
    map<string, vector<cv::Rect2i>> cp2_mmDetectMap;


};


#endif //YOLO_DETECT_H