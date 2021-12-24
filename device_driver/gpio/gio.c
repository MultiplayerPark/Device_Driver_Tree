/*
 * File Name : gio.c
 *
 * Copyright 2021 by pwk
 *
 * Developer : PWK (pwk10000@naver.com)
 *
 * Classify : Device-Driver
 *
 * Version : 1.00
 *
 * Created Date : 2021-12-24
 *
 * File Description : GPIO Control Driver(Linux-3.0.xx)
 *
 * Release List
 * 2021-12-24 : 1st Release
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>

/* by PWK 2017.04.13
 * */
//#define GPIO_OUTPUT_DIR		_IOW('g',0x01,int)	// gpio pin direction setting output
//#define GPIO_INPUT_DIR		_IOW('g',0x02,int)	// gpio pin direction setting input
//#define GPIO_OUTPUT_HIGH	_IOW('g',0x03,int)	// gpio pin output high
//#define GPIO_OUTPUT_LOW		_IOW('g',0x04,int)	// gpio pin output low

#define GPIO_DIR_SET		_IOW('g',0x01,int)	// gpio pin direction setting output
#define GPIO_OUTPUT_WRITE	_IOW('g',0x02,int)	// gpio pin output high
#define GPIO_INPUT_READ		_IOW('g',0x03,int)	// gpio pin input read value (1:high / 0:low)
#define GPIO_FREE					_IOW('g',0x04,int)	// gpio pin export free

/* by PWK 2017.04.13
 * */
#include <asm/uaccess.h>

typedef struct st_gpioStruct {
	unsigned int gpioPort;
	unsigned int gpioPin;
	unsigned int data;
}gpioStruct_t;

#define GPIO_NR(bank,nr)	((bank - 1) * 32 + nr)
/*
#define IMX_GPIO_DIR	1
#define IMX_GPIO_WRITE	2 
#define IMX_GPIO_READ	3
#define IMX_GPIO_FREE	4
*/
#define GPIO_DIR_OUTPUT 0
#define GPIO_DIR_INPUT	1

#define GPIO_OUT_LOW	0
#define GPIO_OUT_HIGH	1

#define DRV_RET_OK		1
#define DRV_RET_FAIL	-1
// -----------------------------------------------------


static int gio_open(struct inode *minode, struct file *mfile)
{
    return 0;
}

static int gio_release(struct inode *minode, struct file *mfile)
{
    return 0;
}

static int gio_writesmk(struct file *file, unsigned char *buf,
			size_t count, loff_t *f_pos)
{
	return 0;
}

static int gio_read(struct file *file, unsigned char *buf,
            size_t count, loff_t *f_pos)
{

	return 0;
}

// by PWK 2017-04-13
static long gio_ioctl(struct file *file, unsigned int  cmd, unsigned long *arg)
{
	int status;
	int portPin = 0;
	int err;
	gpioStruct_t control;

	err = copy_from_user(&control, (gpioStruct_t *)arg, sizeof(gpioStruct_t));
	if( err != 0 )
	{
		printk("%s,%s - copy_from_user error \r\n",__FILE__,__FUNCTION__);
		return err;
	}

	//printk("Control[%d] Gpio[%d][%d]\r\n",cmd,control.gpioPort,control.gpioPin);
	
	// Port - Pin Mapping...
	portPin = GPIO_NR(control.gpioPort, control.gpioPin);
	
	switch(cmd)
	{
		case GPIO_DIR_SET:
			if( gpio_request(portPin,"ADENG_GPIO") == -EBUSY )
			{
				printk("[Kernel Message] Overlapping Used Pin\r\n");
				return -EBUSY;
			}
			switch( control.data )
			{
				case GPIO_DIR_INPUT :
					status = gpio_direction_input(portPin);	// success is return 0 or fail is return minus number of error number
					break;
				case GPIO_DIR_OUTPUT :
					status = gpio_direction_output(portPin,0);	// success is return 0 or fail is return minus number of error number
					break;
				default :
					return -1;
			}

			if(0 > status)	
			{
				gpio_free(portPin);
			}
			return status;			
			break;
		case GPIO_OUTPUT_WRITE:
			switch( control.data )
			{
				case GPIO_OUT_HIGH :
				case GPIO_OUT_LOW :
		  			gpio_set_value(portPin,control.data);
					break;
				default :
					return -1;
			}
			return 0;
			break;
		case GPIO_INPUT_READ:
			control.data = gpio_get_value(portPin);
			copy_to_user((gpioStruct_t *)arg, &control, sizeof(gpioStruct_t));	
			return status;
			break;

		case GPIO_FREE:
			gpio_free(portPin);
			return 0;
			break;

		default:
			return -1;
	}
}

static struct file_operations gio_fops = {
   .open	=   gio_open,
   .release	=   gio_release,
   .read	=   gio_read,
   .write	= 	gio_writesmk,
   .unlocked_ioctl	= 	gio_ioctl,
};

static int gio_probe(struct platform_device *pdev)
{
	return 0;
}

static int gio_remove(struct platform_device *pdev)
{

	return 0;
}

#ifdef CONFIG_PM
static int gio_suspend(struct platform_device *pdev,
				 pm_message_t state)
{

	return 0;
}

static int gio_resume(struct platform_device *pdev)
{

	return 0;
}
#else
#define gio_suspend	NULL
#define gio_resume	NULL
#endif

static struct platform_driver gio_driver = {
	.driver		= {
		.name	= "gio-ctrl",
		.owner	= THIS_MODULE,
	},
	.probe		= gio_probe,
	.remove		= gio_remove,
	.suspend	= gio_suspend,
	.resume		= gio_resume,
};

static int __init gio_init(void)
{
	int ret = 0;
	struct class *gpio_class;
	struct device *dev;

	ret = register_chrdev(222, "gio", &gio_fops);
	if(ret<0) printk(" gpio char dev fail !!!!\r\n");

	gpio_class = class_create(THIS_MODULE, "gio");
	dev = device_create(gpio_class, NULL, MKDEV(222,0), NULL, "gio");

	printk("gio dev ret= %d \r\n", ret);
	return ret;
}
module_init(gio_init);

static void __exit gio_exit(void)
{

}
module_exit(gio_exit);

MODULE_DESCRIPTION("G P I O Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gio-ctrl");

