#ifndef __NCNN_MODEL__
#define __NCNN_MODEL__

#include "layer.h"
#include "net.h"

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <float.h>

//输出物体位置的结构体
struct Object
{
    cv::Rect_<float> rect; //物体边框
    int label;             //物体类型
    float prob;            //置信度
};

//yolo11-核心函数
int NCNN_NetLoad(char* param_path, char* bin_path, int cpuNum);
int NCNN_Pre(const cv::Mat& bgr);
int NCNN_NetRun(int out_ix);
int NCNN_Post(std::vector<Object>& objects, int out_ix);

#endif //__NCNN_MODEL__