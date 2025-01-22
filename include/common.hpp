#ifndef __COMMON__
#define __COMMON__

/****** 通用函数库 ********/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <typeinfo>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/socket.h>  //socket 通信
#include <netinet/in.h> 
#include <arpa/inet.h>

#include <pthread.h> //配置线程
#include <sched.h> //配置进程 调度方式

#include <sys/ipc.h> //进程间通信
#include <sys/shm.h> //共享内存

#include <fcntl.h> //读写设备，文件相关函数（硬件驱动）
#include <sys/ioctl.h>

#include <Python.h> //添加python头文件

#include <Eigen/Core> //添加Eigen库
#include <Eigen/Dense> //稠密矩阵的计算

/*****时间管理模块 ******/
// 需要打开驱动设备 timer_space 计时设备

/*
 * 描述：时间管理模块的设备初始化
 * 输入：无
 * 输出：时间管理设备fid
 */
extern int timer_init(void);
/*
 * 描述：校时函数，校正计时器时间
 * 输入：所要校正的时间/ms
 * 输出：校时是否成功标志
 */
extern int set_time_ms(unsigned long long time);
/*
 * 描述：获得距离校时时刻的ms数，未校时，时间原点为设备开机时刻
 * 输入：所要校正的时间/ms
 * 输出：校时是否成功标志
 */
extern unsigned long long get_time_ms(void);
/*
 * 描述：延时函数
 * 输入：所要延迟的时间/ms
 * 输出：无
 */
extern void delay_ms(unsigned long ms);
/*
 * 描述：等待时间到ms
 * 输入：所要到达的时间ms
 * 输出：无
 */
extern void Wait2Time_ms(unsigned long ms);

#endif //__COMMON__
