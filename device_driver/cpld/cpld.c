/*
 * File Name : cpld.c
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
 * File Description : CPLD(Xlinx) Control Device-Driver(Linux)
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
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

//================================================//
#define DEVICE_NAME "cpld_drv"
#define MAJOR_NUMBER 224
#define DRIVER_VERSION "1.00"
#define DRIVER_DESC "CPLD Driver Version "DRIVER_VERSION
//================================================//

#define CPLD_PHY_ADDRESS 0xC000000
#define CPLD_REMAP_SIZE 0x1000000

#define CPLD_OFFSET_0 0x0
#define CPLD_OFFSET_1 0x100000
#define CPLD_OFFSET_2 0x200000
#define CPLD_OFFSET_3 0x300000
#define CPLD_OFFSET_4 0x400000
//===============================================//

typedef struct cpldControlData{
	unsigned int reg;
	unsigned int data;
}cpldControlData_t;

unsigned char *cpldBase;

unsigned int cpldreg;
//================================================//

#define CPLD_READ     _IOW('k',0x01,cpldControlData_t)
#define CPLD_WRITE    _IOW('k',0x02,cpldControlData_t)

#define CPLD_100       1
#define CPLD_200       2
#define CPLD_300       3
#define CPLD_400       4
#define CPLD_500       5
//================================================//


static int memoryAllocation(void)
{
	cpldBase = (unsigned char *)ioremap(CPLD_PHY_ADDRESS,CPLD_REMAP_SIZE);
	if(cpldBase == NULL)
	{
		printk("cpld memory allocation fail \n");
		return -1;
	}
	printk("CPLD Base Address:0x%08x is mapped to 0x%08x\n",CPLD_PHY_ADDRESS,(unsigned int)cpldBase);
	return 0;
}
static int cpld_open(struct inode *minode, struct file *mfile)
{
    return 0;
}

static int cpld_release(struct inode *minode, struct file *mfile)
{
//		printk("cpld release!! \n");
    return 0;
}

static int cpld_write(struct file *file, const char *buf,
			size_t count, loff_t *f_pos)
{
//	printk("cpld write!! \n");
	return 0;
}

static int cpld_read(struct file *file,  char *buf,
            size_t count, loff_t *f_pos)
{

	return 0;
}

static long cpld_ioctl(struct file *file, unsigned int  cmd, unsigned long arg)
{
	int status;
	int size,err=0;

	printk("CPLD Base Address:0x%08x is mapped to 0x%08x\n",CPLD_PHY_ADDRESS,(unsigned int)cpldBase);
	cpldControlData_t control;
	// Check...
	if(_IOC_TYPE(cmd) != 'k'){
		printk("++++\n");
		return -EINVAL;
	}
	size = _IOC_SIZE(cmd);
	if(size){
		err=0;

		if(_IOC_DIR(cmd) & _IOC_READ)
			err = !access_ok(VERIFY_WRITE,(void __user *)arg,size);
		else if(_IOC_DIR(cmd) & _IOC_WRITE)
			err = !access_ok(VERIFY_READ,(void __user *)arg,size);
		if(err)return -EFAULT;
	}
	
	status = copy_from_user(&control,(cpldControlData_t *)arg,sizeof(cpldControlData_t));
	if(status != 0)
	{
		printk("%s,%s- copy_from-user error \n",__FILE__,__FUNCTION__);
		return status;
	}

	printk("cpld cmd[%d]\r\n",cmd);

	switch(cmd)
	{
		case CPLD_READ:
			switch(control.reg){
				case CPLD_100:
					control.data = readb(cpldBase + CPLD_OFFSET_0);
					break;
				case CPLD_200:
					control.data = readb(cpldBase + CPLD_OFFSET_1);
					break;
				case CPLD_300:
					control.data = readb(cpldBase + CPLD_OFFSET_2);
					break;
				case CPLD_400:
					control.data = readb(cpldBase + CPLD_OFFSET_3);
					break;
				case CPLD_500:
					control.data = readb(cpldBase + CPLD_OFFSET_4);
					break;
				default:
					return -1;
			}
			status = copy_to_user((cpldControlData_t *)arg,&control,sizeof(cpldControlData_t));
			if(status != 0)
			{
				printk("%s,%s- copy_to_user error \n",__FILE__,__FUNCTION__);
				return status;
			}
			break;

		case CPLD_WRITE:
			switch(control.reg){
				case CPLD_100:
					writeb(control.data,cpldBase + CPLD_OFFSET_0);
//					printk("In CPLD100\n");
					break;
				case CPLD_200:
					writeb(control.data,cpldBase + CPLD_OFFSET_1);
//					printk("In CPLD200\n");
					break;
				case CPLD_300:
					writeb(control.data,cpldBase + CPLD_OFFSET_2);
//					printk("In CPLD300\n");
					break;
				case CPLD_400:
					writeb(control.data,cpldBase + CPLD_OFFSET_3);
//					printk("In CPLD400\n");
					break;
				case CPLD_500:
					writeb(control.data,cpldBase + CPLD_OFFSET_4);
//					printk("In CPLD500\n");
					break;
				default:
					printk("Unknown[%d]\n",control.reg);
					return -1;
			} //end write switch
			break;

		default:
			return -1;
			break;
	} //end switch
	return 0;
}

static struct file_operations cpld_fops = {
   .read	=   cpld_read,
   .write	= 	cpld_write,
   .unlocked_ioctl	= 	cpld_ioctl,
   .open	=   cpld_open,
   .release	=   cpld_release,
};

static int __init cpld_init(void)
{
	int ret = 0;
	char *version = DRIVER_VERSION;
	struct class *cpld_class;
	struct device *dev;

	ret = register_chrdev(MAJOR_NUMBER, DEVICE_NAME, &cpld_fops);
	if(ret<0) printk(" cpld char dev fail !!!!\r\n");

	cpld_class = class_create(THIS_MODULE, DEVICE_NAME);
	dev = device_create(cpld_class, NULL, MKDEV(224,0), NULL, DEVICE_NAME);

	printk("\n");
	printk("--------------------------Loading CPLD Module -------------------------\n");
	printk("CPLD Device registered with Major Number = %d\n", MAJOR_NUMBER);
	printk("D/D Version       : %s \n", version);
	memoryAllocation();
	printk("Build Time        : %s %s \n", __DATE__,__TIME__);
	printk("-----------------------------------------------------------------------\n");
	printk("cpld dev ret= %d \r\n", ret);
	
	printk("led:0x%08x is mapped to 0x%08x\n",CPLD_PHY_ADDRESS,(unsigned int)cpldBase+ CPLD_OFFSET_2);
	return ret;
}
module_init(cpld_init);

static void __exit cpld_exit(void)
{

}
module_exit(cpld_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("A&D Engineering");

