#include <linux/types.h> //设备号相关函数
#include <linux/kernel.h> //内核核心函数
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h> //device，class结构及其相关函数
#include <linux/cdev.h> //字符设备的cdev类及其函数
#include <linux/gpio/consumer.h> //gpio子系统
#include <linux/of.h>   //dts节点相关函数
#include <linux/platform_device.h> //platform驱动相关
#include <linux/errno.h> //相关报错函数

static struct gpio_desc* led_gpio = NULL; //gpio描述符

/*
 * 描述：platform驱动的probe函数，当platform_device与platform_driver配对后，该函数执行
 * 参数：platform_device 设备结构体
 * 返回：0,成功；<0 失败
 */
static int gpioLED_probe(struct platform_device *pdev){
	printk("Driver and Device has matched!\r\n");

	led_gpio = gpiod_get(&pdev->dev, "led17", GPIOD_OUT_HIGH);
	if(IS_ERR(led_gpio)){
		dev_err(&pdev->dev, "Failed to get led from device tree\n");
		return PTR_ERR(led_gpio);
	}

	gpiod_set_value(led_gpio, 1);

	return 0;
}

/*
 * 描述：移除platform驱动的时候运行该函数
 * 参数：platform_device 设备结构体
 * 返回：0 成功；<0 失败
 */
static int gpioLED_remove(struct platform_device* dev){
	gpiod_put(led_gpio);
	printk("GPIO LED Closed!");
	return 0;
}

/*
 * 设备树中compatible数值，配对使用
 */
static const struct of_device_id rpi_led_table[] = {
    { .compatible = "led_gpio17" },
    { },
};

/*
 * platform driver 驱动结构体
 */
static struct platform_driver gpioLED = {
		.driver = {
				.name = "gpioLed17",  /*该驱动的名字，用于与设备匹配*/
				.of_match_table = rpi_led_table, /*rpi 设备树*/
		},
		.probe = gpioLED_probe,
		.remove = gpioLED_remove,
};

/*
 * 描述：驱动模块加载
 * 参数：无
 * 返回：无
 */
static int __init gpioLED_init(void){
	return platform_driver_register(&gpioLED);
}

/*
 *描述：驱动模块卸载函数
 *参数：无
 *返回：无
 */
static void __exit gpioLED_exit(void){
	platform_driver_unregister(&gpioLED);
}

module_init(gpioLED_init);
module_exit(gpioLED_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("kdp");


