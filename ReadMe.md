# Embeded Space-AI Framework 

本项目致力于构建一个高鲁棒性且对智能算法开发友好的嵌入式框架（Embeded Space-AI Framework）。该框架采用基于多核处理器的冗余安全设计，保障系统在极端环境下的稳定运行；同时，该框架的开发基于嵌入式Linux系统，支持现有主流AI算法框架（tensorflow / pytorch等），保证创意性AI算法的快速迁移与实现。

Eembeded Space-AI Framework具有以下特点：

* 精确至ms的时钟管理模块
* 高效的多核冗余安全架构
* 丰富的对外通讯接口模块
* 可快速部署AI算法的系统环境

该项目已分享至 https://github.com/KDP777/Embedded-Space-AI-Framework

## 时钟管理模块

该模块采用1000Hz的Linux内核时钟，计时精度1ms。

同时，该系统支持Xenomai-4的EVL实时内核，可提供更实时的线程抢断响应。

该时钟管理模块经过了cyclictest和latmus的响应时间测试（测试详情见后）。Linux内核最低响应延时为14us，平均响应延时为 19us，最高响应延时为81us；EVL实时内核最低响应延时为2.179us，平均响应延时为4.042us，最高响应延时为20.997us；两个内核的响应延时均$ \leq $ 100us，可满足大部分的任务需求。

时钟管理模块提供以下功能：

- 时钟校正功能
- 精确延时功能（延时精确度优于usleep系列函数）
- 当前时间读取功能

详情请见/include/common.hpp中timer模块。

## 独占CPU的任务设计

该框架下的所有运行任务均采用Linux SCHED_FIFO的调度方式，并把任务的优先级提到最高，使其独占CPU。

采用独占CPU的任务设计是基于安全方面的考虑，可避免系统底层任务切换带来的总体性风险，同时独占CPU的进程可以最大程度确保对外部输入的实时响应。

该任务调度设计有如下特点：

* 一个进程独占一个CPU核心，确保任务完成的安全性和实时性；
* 各个进程采用SCHED_FIFO调度方式，并将任务优先级提为最高的99；
* 多个进程分别运行在多个CPU核心上，通过共享内存进行IPC交互；



## 多核冗余安全架构

该架构的设计原理类似于生物DNA的表达模式。单个细胞的DNA序列包含生物体的全部信息，然而正常情况下，单个细胞只是选择性表达了DNA序列的部分信息，以完成与其他细胞的高效合作；当某一细胞失能后，其他细胞可通过完整的DNA序列信息复刻该细胞的工作职能，保障该生物系统的稳定运行。

现有的大多数嵌入式处理器都具有多个cpu核心（4核，6核，8核等）。本架构的每一个cpu核心的进程都拥有完整的任务代码，类似于生物体的全套DNA信息。当所有cpu都正常工作时，各个cpu只会唤醒代码中的部分功能，多个cpu相互协作以高效实现任务代码的全部功能。但在极端环境下，某个cpu一旦失能，其他cpu可通过查询共享内存广播数据段中的cpu任务运行节拍数确定该cpu已无法工作，从而重新进行任务分配，其他正常工作的cpu将承担失能cpu的功能。

该架构的示意图如下所示：

<img src="https://image-kdp777.obs.cn-north-4.myhuaweicloud.com/cpusche.png" alt="多核CPU工作框架图" style="zoom: 50%;" />

多核间的共享内存数据段承担着cpu核心间的通信工作，是该架构的核心组成部分，因此采用三取二的数据保护措施。该安全冗余的架构设计使得各个cpu核心即是相互协作的关系，又是相互备份的关系，兼顾了系统框架的高效运行与稳定安全的需求。

### 共享内存数据段

共享内存是各个进程交互数据的主要手段，其主要的设计如下：

* 对于共享内存中某个数据段，只有一个进程拥有写的权力，其他进程只能进行读操作；
* 当某个CPU失能后，后续接手的CPU将继承其共享内存的写权限；
* 所有数据段均采用三取二的冗余设计；

共享内存段根据功能分类，主要由以下几个部分构成：CPU任务运行节拍数，软件任务基本配置项，CPU核间数据交换缓存空间。

### CPU任务运行节拍数

该框架的整体设计思路引入了嵌入式常见的时序设计，各个独占CPU的进程在一个时间周期中完成特定的任务，任务执行结束后，将CPU对应的节拍数进行+1的操作，并存放在共享内存对应的位置中。

CPU任务运行节拍数有如下作用：

* 同步各个实时进程的运行周期；
* 进程正常运行的标志：各个进程可通过查询其他进程的节拍数，判断其是否正常；
* 作为调度各进程间运行时序的依据；

### 软件任务基本配置项

存储框架中各进程运行中的软件配置项，确保某进程失能后，其他cpu核心可通过该配置项接管任务。

### CPU核间数据交换缓存空间

为各个进程之间的数据交换提供支持，确保各个进程进行高效的数据交互。

### 算法模块（algorithm_module类）与cpu核心的关系

在该框架中，算法模块封装了algorithm_module类进行进程间的任务交互，详见/include/proc.h。

algorithm_module类中集成了进程间合作交互的必要变量（模块运行CPU，共享内存地址等等），同时也提供了具体函数运行的接口；algorithm_module初始化默认算法模块不在该进程的cpu上运行，需要调用activate函数才会将该算法模块加入进程cpu的运行序列。

一个algorithm_module类的实例唯一对应某个CPU核心，但单个CPU核心可对应多个algorithm_module类的实例，具体根据任务实际需求进行设计。

建议：正常情况下，单个CPU对应一个algorithm_module实例，故障情况下，再使用单CPU运行多algorithm_module实例的设计。



## 通讯接口模块

该框架支持Linux内核提供的所有板间通讯协议，主要的通信接口如下：

- UART串口通讯（详见/include/uart.hpp）
- I2C通讯
- SPI通讯
- USB通讯



## AI算法环境部署

本框架基于嵌入式Linux内核架构，支持pytorch，tensorfolow等通用AI算法框架的部署，可实现AI算法的快速开发。



## 硬件平台，Linux内核版本

硬件平台：RaspberryPi 4B

Linux内核版本：Xenomai 4 （双内核架构：Linux kernel + EVL RT kernel）



## 开发语言

User Space：C++，python；

Kernel Space ：C （相关驱动开发）



## Demo

1. PID算法轨道控制，见/user/PID_OrbitCtl ;
2. 陨坑导航算法，见/user/CraterNav.;



具体算法程序开发请参见/usr/usr_template.cpp

程序运行需要用超级管理员权限，程序输入参数如下：

  1.输入1：该进程对应的算法模块序号（从0开始）
  3.输入2：该进程使用的cpu核心（从0开始）
Ubuntu示例如下：

```
cd ./build/
sudo ./Space_AI 0 2 (算法模块0，使用CPU2运行)
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
