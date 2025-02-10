#include <linux/types.h> //设备号相关函数
#include <linux/kernel.h> //内核核心函数
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h> //device，class结构及其相关函数
#include <linux/cdev.h> //字符设备的cdev类及其函数
#include <linux/of.h>   //dts节点相关函数
#include <linux/jiffies.h> //系统的时间计数器，1000Hz
#include <linux/delay.h> //系统延时函数

#define DEV_CNT 1       //设备号数量
#define TIMER_NAME "timer_space"
#define US_DELAY 0
#define WAIT_ToTime 1

struct timer_dev{
	dev_t devid;                 //设备号
	int major;                   //主设备号
	int minor;                   //次设备号
	struct cdev cdev;            //字符设备 cdev
	struct class* class;         //设备类
	struct device* device;       //设备
	struct device_node* nd;      //设备节点
};

struct timer_dev timerDev;       /*timer 设备*/
unsigned long long time_bias;
unsigned long long origin;

/*
 * 描述：打开设备
 * 参数： inode-传递给驱动的inode， flip-设备文件，file结构体有个叫做private_data的成员变量，
 * 		 一般在open的时候将private_data指向设备结构体
 * 返回：0 成功， 其他失败
 */
static int timer_open(struct inode* inode, struct file* flip){
	flip->private_data = &timerDev; //设置私有数据
	time_bias = 0;
	origin = 0;

	return 0;
}

/*
 * 描述：读取当前时间，相较于基准时间的延时/ms
 * 参数： cur_time-相对于基准时间的ms延时
 * 返回：0 成功， 其他失败
 */
static ssize_t timer_read(struct file* flip, char* __user cur_time,
							size_t cnt, loff_t* offt){
	unsigned long long time;
//	char[8] time_tmp;
	ssize_t ret = 0;

	time = jiffies - origin + time_bias;

//	// unsigned long 转为 char[8]
//	time_tmp[0] = time>>56;
//	time_tmp[1] = time>>48;
//	time_tmp[2] = time>>40;
//	time_tmp[3] = time>>32;
//	time_tmp[4] = time>>24;
//	time_tmp[5] = time>>16;
//	time_tmp[6] = time>>8;
//	time_tmp[7] = time;

	ret = copy_to_user(cur_time, &time, 8);

	return ret;
}

/*
 * 描述：设定基准时间/ms
 * 参数：set_time-需要写入的基准时间
 * 返回：0 成功， 其他失败
 */
static ssize_t timer_write(struct file* flip, const char* __user set_time,
							size_t cnt, loff_t* offt){
	unsigned long long time;
	unsigned char time_tmp[8];
	ssize_t ret = 0;

	ret = copy_from_user(time_tmp, set_time, cnt);

	//转化输入的char* 为 unsigned long long time
	time = ((unsigned long long)time_tmp[0]) |
		   ((unsigned long long)time_tmp[1]<<8) |
		   ((unsigned long long)time_tmp[2]<<16) |
		   ((unsigned long long)time_tmp[3]<<24) |
		   ((unsigned long long)time_tmp[4]<<32) |
		   ((unsigned long long)time_tmp[5]<<40) |
		   ((unsigned long long)time_tmp[6]<<48) |
		   ((unsigned long long)time_tmp[7]<<56);

	origin = jiffies_64;

	time_bias = time;

	return ret;
}

static long timer_ioctl(struct file* flip, unsigned int cmd, unsigned long arg){
	unsigned long long i, n, time;
	unsigned long long _ms,_us;

	switch (cmd){
	case US_DELAY:
		if(arg <= 10000){
			udelay(arg);
			return 1;
		}
		else{
			_ms = arg/1000;
			time = jiffies_64+_ms;
			_us = arg-_ms*1000;
			n = (arg/100)+1;
			for (i=0;i<n;i++){
				if(jiffies_64>=time){
					break;
				}
				udelay(100);
			}
			udelay(_us);
		}
		break;
	case WAIT_ToTime:
		_ms = time_bias - origin ;
		if ((jiffies_64+_ms) > arg){
			printk("TIME_WAIT,Please check the input arg, It is the past time!");
			return -1;
		}
		time = arg-(jiffies_64+_ms);
		n = time*100;

		for(i=0;i<n;i++){
			if((jiffies_64+_ms)>=arg){
				break;
			}
			udelay(10);
		}
		break;
	default:
		break;
	}

	return 1;

}

// 设备操作函数
static struct file_operations timer_fops = {
		.owner = THIS_MODULE,
		.open = timer_open,
//		.close = ,
		.read = timer_read,
		.write = timer_write,
		.unlocked_ioctl = timer_ioctl,
};

/*
 * 描述：Linux设备初始化及其注册
 * 参数：无
 * 返回：无
 */
static int __init timer_init(void){
	/* 注册驱动 */
	/*1. 创建设备号*/
	if (timerDev.major) { //定义了设备号
		timerDev.devid = MKDEV(timerDev.major,0);
		register_chrdev_region(timerDev.devid, DEV_CNT, TIMER_NAME);
	}
	else{ //没有定义设备号
		alloc_chrdev_region(&timerDev.devid, 0, DEV_CNT, TIMER_NAME);
		timerDev.major = MAJOR(timerDev.devid); //获得主设备号
		timerDev.minor = MINOR(timerDev.devid); //获得次设备号
	}

	/*2.初始化cdev*/
	timerDev.cdev.owner = THIS_MODULE;
	cdev_init(&timerDev.cdev, &timer_fops);

	/*3.添加一个cdev*/
	cdev_add(&timerDev.cdev,timerDev.devid,DEV_CNT);

	/*4.创建类*/
	timerDev.class = class_create(TIMER_NAME);
	if (IS_ERR(timerDev.class)){
		return PTR_ERR(timerDev.class);
	}

	/*5.创建设备*/
	timerDev.device = device_create(timerDev.class, NULL,
									timerDev.devid,NULL,TIMER_NAME);
	if (IS_ERR(timerDev.device)){
		return PTR_ERR(timerDev.device);
	}

	printk("Timer for Space Opened!\r\n");

	return 0;
}

/*
 * 描述：驱动出口函数
 * 参数: 无
 * 返回： 无
 */
static void __exit timer_exit(void){

	/*注销字符设备驱动*/
	cdev_del(&timerDev.cdev); //删除cdev
	unregister_chrdev_region(timerDev.devid,DEV_CNT);

	/*注销设备*/
	device_destroy(timerDev.class, timerDev.devid);
	class_destroy(timerDev.class);

	printk("Timer for Space Closed!\r\n");

}

module_init(timer_init);
module_exit(timer_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("kdp");


















