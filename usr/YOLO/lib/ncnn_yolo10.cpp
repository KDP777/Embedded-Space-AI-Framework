/*
 *	描述：使用ncnn架构搭建yolo10n网络，并进行推理
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
ncnn::Net yolo10;
// 网络输入
ncnn::Mat in_pad;
int target_size = 640; //输入图像大小
// 网络输出
ncnn::Mat out0;
ncnn::Mat out1;
ncnn::Mat out2;
// 网络参数配置
const float prob_threshold = 0.25f; //设定检测的可信阈值
const float nms_threshold = 0.45f; //设定 Non-Maximum Suppression(NMS) 阈值
float scale = 1.f; //输入图像与原图的放缩比例
int wpad,hpad; //边缘padding大小
int num_labels = 80; //输出物体类别数量
int img_h,img_w; //原始图像的高和宽

/*************************************支持函数**************************************/
/* 关于提取边框的softmax函数
 * 描述：...
 */
static float softmax(
    const float* src,
    float* dst,
    int length
)
{
    float alpha = -FLT_MAX;
    for (int c = 0; c < length; c++)
    {
        float score = src[c];
        if (score > alpha)
        {
            alpha = score;
        }
    }

    float denominator = 0;
    float dis_sum = 0;
    for (int i = 0; i < length; ++i)
    {
        dst[i] = expf(src[i] - alpha);
        denominator += dst[i];
    }
    for (int i = 0; i < length; ++i)
    {
        dst[i] /= denominator;
        dis_sum += i * dst[i];
    }
    return dis_sum;
}

// 基于图像大小裁减物体边框
static float clamp(
    float val,
    float min = 0.f,
    float max = 1280.f
)
{
    return val > min ? (val < max ? val : max) : min;
}

/* 对yolo10输出进行解析
 *
 */
static void output_parse(
    int stride,
    const ncnn::Mat& feat_blob,
    const float prob_threshold,
    std::vector<Object>& objects
)
{
    const int reg_max = 16;
    float dst[16];
    const int num_w = feat_blob.w;
    const int num_grid_y = feat_blob.c;
    const int num_grid_x = feat_blob.h;

    const int num_class = num_w - 4 * reg_max;

    for (int i = 0; i < num_grid_y; i++)
    {
        for (int j = 0; j < num_grid_x; j++)
        {

            const float* matat = feat_blob.channel(i).row(j);

            int class_index = 0;
            float class_score = -FLT_MAX;
            for (int c = 0; c < num_class; c++)
            {
                float score = matat[c];
                if (score > class_score)
                {
                    class_index = c;
                    class_score = score;
                }
            }
            if (class_score >= prob_threshold)
            {

                float x0 = j + 0.5f - softmax(matat + num_class, dst, 16);
                float y0 = i + 0.5f - softmax(matat + num_class + 16, dst, 16);
                float x1 = j + 0.5f + softmax(matat + num_class + 2 * 16, dst, 16);
                float y1 = i + 0.5f + softmax(matat + num_class + 3 * 16, dst, 16);

                x0 *= stride;
                y0 *= stride;
                x1 *= stride;
                y1 *= stride;

                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = x1 - x0;
                obj.rect.height = y1 - y0;
                obj.label = class_index;
                obj.prob = class_score;
                objects.push_back(obj);

            }
        }
    }
}

/*********************************核心函数*****************************************/
//ncnn 网络载入
int NCNN_NetLoad(char* param_path, char* bin_path, int cpuNum){
	//网络配置
    yolo10.opt.use_vulkan_compute = false; //是否使用vulkan加速-rasp4B似乎不支持
    yolo10.opt.num_threads = cpuNum<=MAX_CPUNUM? cpuNum:MAX_CPUNUM; //设置运行的cpu核心

    if (yolo10.load_param(param_path))
        exit(-1);
    if (yolo10.load_model(bin_path))
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
int NCNN_NetRun(int out_ix){

    //out0 - Stride 8
    //out1 - Stride 16
    //out2 - Stride 32
	ncnn::Extractor ex = yolo10.create_extractor();

    ex.input("in0", in_pad); //将padding好的图像输入执行器(extractor)中

    switch(out_ix){
    case 0:
        ex.extract("out0", out0);
        break;
    case 1:
        ex.extract("out1", out1);
        break;
    case 2:
        ex.extract("out2", out2);
        break;
    default:
        cerr<<"NetRUN Error: the out param is not available!"<<endl;
        exit(-1);
    }

    return 1;
}

//ncnn 网络后处理
int NCNN_Post(std::vector<Object>& objects, int out_ix){
	float x0,y0,x1,y1;

    std::vector<Object> objects_out;

    switch(out_ix){
    case 0:
        output_parse(8, out0, prob_threshold, objects_out);
        break;
    case 1:
        output_parse(16, out1, prob_threshold, objects_out);
        break;
    case 2:
        output_parse(32, out2, prob_threshold, objects_out);
        break;
    default:
        cerr<<"Post Error: the out param is not available!"<<endl;
        exit(-1);
    }

    //输出恢复：box坐标恢复至原图大小的尺寸

    for (auto& pro : objects_out)
    {
        x0 = pro.rect.x;
        y0 = pro.rect.y;
        x1 = pro.rect.x + pro.rect.width;
        y1 = pro.rect.y + pro.rect.height;

        x0 = (x0 - (wpad / 2)) / scale;
        y0 = (y0 - (hpad / 2)) / scale;
        x1 = (x1 - (wpad / 2)) / scale;
        y1 = (y1 - (hpad / 2)) / scale;

        x0 = clamp(x0, 0.f, img_w);
        y0 = clamp(y0, 0.f, img_h);
        x1 = clamp(x1, 0.f, img_w);
        y1 = clamp(y1, 0.f, img_h);

        Object obj;
        obj.rect.x = x0;
        obj.rect.y = y0;
        obj.rect.width = x1 - x0;
        obj.rect.height = y1 - y0;
        obj.prob = pro.prob;
        obj.label = pro.label;
        objects.push_back(obj);
    }

    return 1;
}
