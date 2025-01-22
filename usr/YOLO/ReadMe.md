## YOLO10n实时物体检测

**硬件平台：**RaspberryPi 4B，设置4核cpu中cpu 0,1为A计算单元，cpu 2,3为B计算单元； USB摄像头-实时采集图像；

<img src="https://image-kdp777.obs.cn-north-4.myhuaweicloud.com/img/YOLO_HardWare.jpg" style="zoom: 33%;" />

​        连接网线的目的是将RaspberryPi 4B解算结果通过UDP/IP上传到PC上实时显示，白色摄像头通过USB与RaspberryPi连接。

**算法简介：**

1. 摄像头连接到RaspberryPi 4B上，实时采集图像；
2. 采用ncnn框架将YOLO10n部署到RaspberryPi 4B的4核CPU上；
3. YOLO10n对实时采集的图像进行物体识别，并通过UDP/IP上传到PC显示；

**开发语言：**C++；

**架构设计：**

* 各模块正常工作：

  1）A机：采集摄像头图像，运行YOLO10n Stride=16的网络模块，计算物体识别方框；

  2）B机：运行YOLO10n Stride=32的网络模块，计算物体识别方框；

  该设计采用并行模式，YOLO10n Stride=16/32 在 A/B机上并行计算。

* A机或B机失能：

​	A/B机串行计算Stride=16和32的网络推理，运行时间有所增加，需将更新周期拉长。

**Demo演示结果： **

​	左边为A，B机运行时间的展示，右边为实时展示的物体识别效果，运行demo视频请见https://video-share.obs.cn-north-4.myhuaweicloud.com/Demo%20Space-AI/YOLOv10_demo.mp4

![](https://image-kdp777.obs.cn-north-4.myhuaweicloud.com/img/YOLOv10.png)