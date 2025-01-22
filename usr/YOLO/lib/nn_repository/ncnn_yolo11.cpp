/*
 *	描述：使用ncnn架构搭建yolo11n网络，并进行推理
 *	作者：KDP777,参考ncnn官方example/yolov8.cpp
 *	核心函数： 
 *	1) NCNN_NetLoad(char* parm_path, char* bin_path); ncnn网络加载
 *	2) NCNN_Pre(const cv::Mat& bgr); 预处理，将输入图像 scale，padding 至640x640大小
 *	3) NCNN_NetRun(char* output_para, int num_labels); 获得output_para的输出，并放在本文件的全局变量out中
 *	4) NCNN_Post(std::vector<Object>& objects); 后处理，基于out0应用nms等后处理，将结果放在objects中
 */

#include "ncnn_model.h"

using namespace std;

/*********************************全局变量定义**************************************/
// 设最大的stride
#define MAX_STRIDE 32 //CNN的stride
// 设置最大CPU核心数
#define MAX_CPUNUM 4 

// ncnn网络
ncnn::Net yolo11;
// 网络输入
ncnn::Mat in_pad;
int target_size = 640; //输入图像大小
// 网络输出
ncnn::Mat out;
// 网络参数配置
const float prob_threshold = 0.25f; //设定检测的可信阈值
const float nms_threshold = 0.45f; //设定 Non-Maximum Suppression(NMS) 阈值
float scale = 1.f; //输入图像与原图的放缩比例
int wpad,hpad; //边缘padding大小
int num_labels=80; //输出物体类别数量
int img_h,img_w; //原始图像的高和宽

/*************************************支持函数**************************************/

/* 
 * 快速排序-二分法
 */
static void qsort_descent_inplace(std::vector<Object>& objects, int left, int right)
{
    int i = left;
    int j = right;
    float p = objects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (objects[i].prob > p)
            i++;

        while (objects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(objects[i], objects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(objects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(objects, i, right);
        }
    }
}
// 快速排序函数(重构)-二分法
static void qsort_descent_inplace(std::vector<Object>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

/* 
 * Non-Maximun Suppression 非极大值抑制 
 */
// 求两个提取框的交叉面积
static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}
// NMS函数
static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, bool agnostic = false)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            if (!agnostic && a.label != b.label)
                continue;

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

/*
 * 解析 ncnn 网络输出结果 out(8400x84)
 * 检测物体特性： num_channels = 84, 即box 4元素+num_labels
 * 检测物体数量： num_anchors = 8400, 即(640/32)^2+(640/16)^2+(640/8)^2, stride 8 16 32的全部输出
 */ 

// 基于图像大小裁减物体边框
static inline float clampf(float d, float min, float max)
{
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

// 解析out
static void parse_yolo11_detections(
    float* inputs, float confidence_threshold,
    int num_channels, int num_anchors, int num_labels,
    int infer_img_width, int infer_img_height,
    std::vector<Object>& objects)
{

    std::vector<Object> detections;
    cv::Mat output = cv::Mat((int)num_channels, (int)num_anchors, CV_32F, inputs).t();

    for (int i = 0; i < num_anchors; i++)
    {
        const float* row_ptr = output.row(i).ptr<float>();
        const float* bboxes_ptr = row_ptr;
        const float* scores_ptr = row_ptr + 4;
        const float* max_s_ptr = std::max_element(scores_ptr, scores_ptr + num_labels);
        float score = *max_s_ptr;

        if (score > confidence_threshold)
        {
            float x = *bboxes_ptr++;
            float y = *bboxes_ptr++;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr;

            float x0 = clampf((x - 0.5f * w), 0.f, (float)infer_img_width);
            float y0 = clampf((y - 0.5f * h), 0.f, (float)infer_img_height);
            float x1 = clampf((x + 0.5f * w), 0.f, (float)infer_img_width);
            float y1 = clampf((y + 0.5f * h), 0.f, (float)infer_img_height);

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;
            Object object;
            object.label = max_s_ptr - scores_ptr;
            object.prob = score;
            object.rect = bbox;
            detections.push_back(object);
        }
    }
    objects = detections;
}

/*********************************核心函数*****************************************/
//ncnn 网络载入
int NCNN_NetLoad(char* param_path, char* bin_path, int cpuNum){
	//网络配置
    yolo11.opt.use_vulkan_compute = false; //是否使用vulkan加速-rasp4B似乎不支持
    yolo11.opt.num_threads = cpuNum<=MAX_CPUNUM? cpuNum:MAX_CPUNUM; //设置运行的cpu核心

    if (yolo11.load_param(param_path))
        exit(-1);
    if (yolo11.load_model(bin_path))
        exit(-1);

    cout<<"### NCNN Model Load Success!###"<<endl;

    return 1;
}

//ncnn 网络预处理
int NCNN_Pre(const cv::Mat& bgr){
	img_w = bgr.cols;
    img_h = bgr.rows;

    // 1. 输入图像resize
    // 计算resize的尺度信息
    int w = img_w;
    int h = img_h;

    if (w > h)
    {
        scale = (float)target_size / w;
        w = target_size;
        h = h * scale;
    }
    else
    {
        scale = (float)target_size / h;
        h = target_size;
        w = w * scale;
    }

    // 图像resize成(xxx, 640)或(640,xxx)的图像大小
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, w, h);

    // 2.图像padding成640x640大小的图
    wpad = (target_size + MAX_STRIDE - 1) / MAX_STRIDE * MAX_STRIDE - w;
    hpad = (target_size + MAX_STRIDE - 1) / MAX_STRIDE * MAX_STRIDE - h;
    ncnn::copy_make_border(in, in_pad,  //输入输出
                            hpad / 2,                //图像top上方要padding的行数
                            hpad - hpad / 2,         //图像bottom下方要padding的行数
                            wpad / 2,                //图像left左边要padding的列数
                            wpad - wpad / 2,         //图像right右边要padding的列数
                            ncnn::BORDER_CONSTANT, 
                            114.f);


    // 3.in_pad图像 归一化
    const float norm_vals[3] = { 1 / 255.f, 1 / 255.f, 1 / 255.f };
    in_pad.substract_mean_normalize(0, norm_vals);

    return 1;
}

//ncnn 网络运行
int NCNN_NetRun(char* ex_out){

	ncnn::Extractor ex = yolo11.create_extractor();

    ex.input("in0", in_pad); //将padding好的图像输入执行器(extractor)中

    // 输出out(输出已经包含了stride 8, 16, 32)
    ex.extract(ex_out, out);

    return 1;
}

//ncnn 网络后处理
int NCNN_Post(std::vector<Object>& objects){
	float x0,y0,x1,y1;

    std::vector<Object> objects32;
    parse_yolo11_detections(
        (float*)out.data, prob_threshold,
        out.h, out.w, num_labels,
        in_pad.w, in_pad.h,
        objects32);

    // 二分法排序proposals： sort all proposals by score from highest to lowest 
    qsort_descent_inplace(objects32);

    // 非极大值抑制： apply nms with nms_threshold
    std::vector<int> picked;
    nms_sorted_bboxes(objects32, picked);

    //输出恢复：box坐标恢复至原图大小的尺寸
    int count = picked.size();
    objects.resize(count);

    for (int i = 0; i < count; i++)
    {
        objects[i] = objects32[picked[i]];

        // adjust offset to original unpadded
        x0 = (objects[i].rect.x - (wpad / 2)) / scale;
        y0 = (objects[i].rect.y - (hpad / 2)) / scale;
        x1 = (objects[i].rect.x + objects[i].rect.width - (wpad / 2)) / scale;
        y1 = (objects[i].rect.y + objects[i].rect.height - (hpad / 2)) / scale;

        // clamp
        x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        //将最终结果放入object中
        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }

    return 1;
}
