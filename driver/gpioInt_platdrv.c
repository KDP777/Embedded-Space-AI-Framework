#include <linux/types.h> //设备号相关函数
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h> //device，class结构及其相关函数
#include <linux/cdev.h> //字符设备的cdev类及其函数
#include <linux/gpio.h> //gpio子系统
#include <linux/of.h>   //dts节点相关函数
#include <linux/platform_device.h> //platform驱动相关

#include <linux/interrupt.h> //中断相关函数及其结构体
#include <linux/irq.h>

static struct gpio_desc* int_gpio = NULL; //gpio描述符

/*
 * 特定设备irq结构体
 */
struct gpioIRQ_dev{
	dev_t devid;        //设备号
	struct cdev cdev;    //字符设备结构体
	struct class* class;  //设备类
	struct device* device;  //设备节点结构体
	int major;          //主设备号
	int minor;          //次设备号
	struct device_node *nd; //设备节点
};

struct gpioIRQ_dev gpioIRQ_1;

int irqnum; //中断号
unsigned long irqflags; //中断响应标志位
//irqreturn_t (*isr)(int, void*);
irq_handler_t isr; //中断服务函数

/*
 * 描述：GPIO中断响应函数
 * 参数：中断号，和输入的设备类型
 * 返回：中断执行结果
 */
static irqreturn_t gpioInt_isr(int irq, void *dev_id){
	printk("Int Success!\r\n");

	return IRQ_HANDLED;
}

/*
 * 描述：platform驱动的probe函数，当platform_device与platform_driver配对后，该函数执行
 * 参数：platform_device 设备结构体
 * 返回：0,成功；<0 失败
 */
static int gpioInt_probe(struct platform_device *pdev){
	int ret;
	printk("Driver and Device has matched!\r\n");

	// 设置gpio为INPUT模式
	int_gpio = gpiod_get(&pdev->dev, "int", GPIOD_IN);

	if(IS_ERR(int_gpio)){
		dev_err(&pdev->dev, "Failed to get int gpio from device tree\n");
		return PTR_ERR(int_gpio);
	}

	// 绑定gpio到中断
	irqnum = gpiod_to_irq(int_gpio);

	// 设定中断服务函数
	isr = gpioInt_isr;

	//设定中断响应标志位
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING; //上升沿和下降沿同时响应

	//申请中断
	ret = request_irq(irqnum, isr, irqflags, "GPIOInt1", &gpioIRQ_1);

	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to claim irq %d; error %d\n",
				irqnum, ret);
		return ret;
	}

	return 0;
}

/*
 * 描述：移除platform驱动的时候运行该函数
 * 参数：platform_device 设备结构体
 * 返回：0 成功；<0 失败
 */
static int gpioInt_remove(struct platform_device* dev){
	//释放中断
	free_irq(irqnum, &gpioIRQ_1);
	//释放gpio口
	gpiod_put(int_gpio);

	return 0;
}

/*
 * 设备树中compatible数值，配对使用
 */
static const struct of_device_id rpi_led_table[] = {
    { .compatible = "gpioInt_key" },
    { },
};


/*
 * platform driver 驱动结构体
 */
static struct platform_driver gpioInt = {
		.driver = {
				.name = "gpioInt",  /*该驱动的名字，用于与设备匹配*/
				.of_match_table = rpi_led_table, /*rpi 设备树匹配*/
		},
		.probe = gpioInt_probe,
		.remove = gpioInt_remove,
};

/*
 * 描述：驱动模块加载
 * 参数：无
 * 返回：无
 */
static int __init gpioInt_init(void){
	return platform_driver_register(&gpioInt);
}

/*
 *描述：驱动模块卸载函数
 *参数：无
 *返回：无
 */
static void __exit gpioInt_exit(void){
	platform_driver_unregister(&gpioInt);
}

module_init(gpioInt_init);
module_exit(gpioInt_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("kdp");


