## Demo-PID轨道控制

**硬件平台：**RaspberryPi 4B，设置cpu 2为A计算单元，cpu 3为B计算单元；

<img src="https://image-kdp777.obs.cn-north-4.myhuaweicloud.com/img/OrbitCtl_HardWare.jpg" style="zoom: 33%;" />

   连接网线的目的是将RaspberryPi 4B解算数据上传到PC进行可视化展示，敏感器采集数据使用文件读取的方式获得。

**算法简介：**

1. 获取StarLink某一颗卫星的TLE(Two-Line Element)轨道参数文件；

2. 生成了UTC时间下卫星在地固系下的xyz三轴位置，作为设计轨道的位置输入；

3. 该Demo模仿实时采集敏感器的数据（IMU的加速度，速度等），解算出当前飞行器的位置；

4. 通过PID控制算法操控飞行器运行在提前装订好的设计轨道上；

**开发语言：**C++；

**架构设计：**

* 各模块正常工作：

  1）A机：模仿采集敏感器数据（IMU输出速度，加速度），解算当前飞行器位置；

  2）B机：基于A机输出的飞行器位置，采用PID控制算法计算出控制飞行器抵达设计轨道所需的加速度；

  该设计采用串行模式，B机需要等到A机运行完成后开始计算。

* A机或B机失能：

​	正常工作的机器接替另一个机器的工作，继续运行。

**Demo演示结果： **

​	蓝色轨道为设计轨道，橘红色轨道为PID算法控制的运行轨道，轨道偏差如右上角所示，运行全流程请见视频https://video-share.obs.cn-north-4.myhuaweicloud.com/Demo%20Space-AI/PIDCtl_demo.mp4<img src="https://image-kdp777.obs.cn-north-4.myhuaweicloud.com/img/PIDCtl.png" style="zoom:67%;" />



