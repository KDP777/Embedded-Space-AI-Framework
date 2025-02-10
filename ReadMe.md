# Embeded Space-AI Framework 

本项目致力于构建一个鲁棒性高且对智能算法开发友好的嵌入式框架（Embeded Space-AI Framework）。该框架采用多机高效合作，相互备份的冗余设计，确保系统在极端环境下能稳健地进行高性能计算；该框架的开发基于嵌入式Linux系统，支持现有主流AI算法框架（pytorch/ncnn/onnx 等），保证创意性AI算法快速迁移实现。

Eembeded Space-AI Framework具有以下特点：

* 精度达到ms的时钟管理模块；
* 各机器正常工作时，多机高效合作最大化算法的运行效率；
* 某机器失能时，多机冗余备份机制启动，确保系统运行；
* 可快速部署AI算法的Linux系统环境；

该项目已分享至 https://github.com/KDP777/Embedded-Space-AI-Framework



## 架构设计

在极端环境下，多机备份已成为常见的安全冗余手段；多个计算单元由于需要相互备份，一般执行相同的任务进程，这无疑浪费了至少一台计算单元的计算性能。在智能算法快速普及的时代，算力的浪费会对神经网络的运行效率造成严重的影响。因此，本架构希望设计一种机制，既能确保多机合作的高性能计算，又能保障极端条件下系统运行的鲁棒性。

该框架的设计原理类似于生物的DNA表达。单个细胞的DNA序列包含生物体的全部信息，然而正常情况下，单个细胞只选择性表达DNA序列的部分信息，与其他细胞的合作，组成一个高效运行的机体；但当某一细胞失能后，其他细胞可通过完整的DNA序列信息获得失能细胞的工作职能，保障生物系统的稳定运行。

本架构的每一个运行的计算单元都拥有完整的任务代码，类似于生物体的全套DNA信息。当所有计算单元都正常工作时，各个计算单元只唤醒代码进程中的部分功能，多机相互协作以高效完成全部任务。但在极端环境下，某个计算单元一旦失能，其他计算单元通过查询共享内存的广播数据段中计算单元运行节拍数确认该计算单元已无法正常工作，依据携带的完整代码重新进行任务分配，其他正常工作的计算单元将接手失能计算单元的工作。

该架构的示意图如下所示：

<img src="https://image-kdp777.obs.cn-north-4.myhuaweicloud.com/img/cpusche.png" alt="cpusche" style="zoom:80%;" />

多机间的共享内存承担着多机合作间的通信工作，是该架构的核心组成部分，因此采用三取二的数据保护措施。该安全冗余的架构设计使得各个计算单元即是相互协作的关系，又是相互备份的关系，兼顾了系统框架的高效运行与稳定安全的需求。

### 独占CPU的进程设计

该框架下的所有计算单元均采用Linux的SCHED_FIFO的进程调度方式，并把任务的优先级提到最高，使其独占CPU的计算，中断资源。

采用独占CPU的任务设计是基于安全方面的考虑，避免了系统底层任务切换带来的总体性风险，同时独占CPU可最大程度确保对外部输入的实时响应。

该任务调度设计有如下特点：

* 一个计算单元与一个用户进程绑定，该进程独占一个CPU核心，确保任务完成的安全性和实时性；
* 进程采用SCHED_FIFO调度方式，并将任务优先级提为最高的99；
* 多个进程分别运行在多个计算单元上，通过共享内存进行IPC交互；

### 共享内存数据段

共享内存是各个计算单元进程交互数据的主要手段，其主要的设计如下：

* 对于共享内存中某个数据段，只有一个进程拥有写的权力，其他进程只能进行读操作；
* 当某个计算单元失能后，后续接手的计算进程将继承其共享内存的写权限；
* 所有数据段均采用三取二的冗余设计；

共享内存段根据功能分类，主要由以下几个部分构成：计算单元任务运行节拍数，软件任务基本配置项，进程间数据交换缓存空间。

### 计算单元任务运行节拍数

该框架的整体设计思路引入了系统控制常见的时序设计，各个计算单元的进程在一个时间周期中完成特定的任务，任务执行结束后，将共享内存中对应的节拍数进行+1的操作。

计算单元任务运行节拍数有如下作用：

* 同步各个实时进程的运行周期；
* 进程正常运行的标志：各个进程可通过查询其他进程的节拍数，判断其是否正常；
* 作为调度各进程间运行时序的依据；

### 软件任务基本配置项

存储各进程软件的关键参数以及运行过程中的输出，确保某进程失能后，其他计算单元可通过配置项接管任务。

### 进程间数据交换缓存空间

为各个进程之间的数据交换提供支持，确保各个进程进行高效的数据交互。

### 算法模块（algorithm_module类）与运行进程的关系

在该框架中，封装了algorithm_module类作为创建各个具体算法模块的基础，详见/include/proc.h。

algorithm_module类中集成了进程间合作交互的必要变量（模块运行CPU，共享内存地址等等），同时也提供了具体函数运行的接口；algorithm_module初始化默认算法模块不在该进程绑定的cpu上运行，需要调用activate函数才会将该算法模块加入该进程的运行序列。

一个algorithm_module类的实例唯一对应某个计算单元的任务进程，但单个计算核心可运行多个algorithm_module 类的实例，具体根据任务实际需求进行设计。

tips：正常情况下，单个计算单元对应一个algorithm_module实例； 故障情况下，使用一个计算单元运行多algorithm_module 实例的设计。



## 时钟管理模块

该模块采用1000Hz的Linux内核高精度时钟，计时精度1ms。

同时，该框架支持Xenomai-4的EVL实时内核，可提供内核态更实时的线程中断响应。

该时钟管理模块经过了cyclictest和latmus的响应时间测试（测试详情见附件）。Linux内核最低响应延时为14us，平均响应延时为 19us，最高响应延时为81us；EVL实时内核最低响应延时为2.179us，平均响应延时为4.042us，最高响应延时为20.997us；两个内核的响应延时均$ \leq $ 100us，可满足大部分的任务需求。

时钟管理模块提供以下功能：

- 时钟校正功能
- 精确延时功能（延时精确度优于usleep系列函数）
- 当前时间读取功能

详情请见/include/common.hpp中timer模块。



## 通讯接口模块

该框架支持Linux内核提供的所有板间通讯协议，主要的通信接口如下：

- UART串口通讯（详见/include/uart.hpp）
- I2C通讯
- SPI通讯
- USB通讯



## 测试硬件平台，Linux内核版本

测试硬件平台：RaspberryPi 4B，设4核cpu中2个cpu为A计算单元，另外2个cpu为B计算单元，支持后续的Demo运行；

Linux内核版本：Xenomai 4 （双内核架构：Linux kernel + EVL RT kernel）



## 开发语言

User Space：C++；

Kernel Space ：C （内核驱动开发）



## Demo

1. PID算法轨道控制，见/usr/PID_OrbitCtl /ReadMe;

   Demo演示视频请见 https://video-share.obs.cn-north-4.myhuaweicloud.com/Demo%20Space-AI/PIDCtl_demo.mp4

   

2. YOLO10n实时物体识别，见/usr/YOLO/ReadMe;

   Demo演示视频请见 https://video-share.obs.cn-north-4.myhuaweicloud.com/Demo%20Space-AI/YOLOv10_demo.mp4

   

算法程序开发模板参见/usr/usr_template.cpp



框架运行步骤：

1. 启动timer_space驱动模块

```
// 编译timer_space模块，注意修改Makefile中的Linux内核路径和当前文件路径
cd ./driver
make
// 添加timer_space模块
sudo cp ./timer_space.ko /lib/modules/***(内核版本号)/
sudo depmod
//加载timer_space驱动
sudo modprobe timer_space
```

2. 启动Space_AI程序

程序输入参数如下：

  输入1：该进程对应的算法模块序号（从0开始）
  输入2：该进程使用的cpu核心（从0开始）

```
cd ./build/
./Space_AI -mod 0 -cpu 2 3 (启动算法模块0，使用CPU2和3运行)
```

## 附件说明

### 时钟管理模块测试

采用cyclictest测试Linux内核响应延时统计如下（24h test）：

>\#  Total: 032033055
>
>\# Min Latencies: 00014
>
>\# Avg Latencies: 00019
>
>\# Max Latencies: 00081

采用latmus测试EVL实时内核响应延时统计如下（24h test）：

>\# libevl version: evl.0.47
>
>\# sampling period: 1000 microseconds
>
>\# clock gravity: 0i 3000k 3000u
>
>\# clocksource: arch_sys_counter
>
>\# vDSO access: architected
>
>\# context: user
>
>\# thread priority: 89
>
>\# thread affinity: CPU2
>
>\# C-state restricted
>
>\# duration (hhmmss): 24:39:22
>
>\# peak (hhmmss): 14:35:16
>
>\# min latency: 2.179
>
>\# avg latency: 4.042
>
>\# max latency: 20.997
>
>\# sample count: 88763340

该测试中的cpu核心均被隔离（isolated）。
