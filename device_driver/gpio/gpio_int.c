#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/gpio.h>

#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include <linux/irq.h>
#include <linux/sched.h>

#define GPIO_NR(bank,nr)	((bank - 1) * 32 + nr)

int portPin = GPIO_NR(5,0);
int gpioint_major = 0, gpioint_minor = 0;
dev_t gpioint_dev;
struct cdev gpioint_cdev;

volatile unsigned int flag_gpioint = 0;
DECLARE_WAIT_QUEUE_HEAD(wq_gpioint);
unsigned int irq_number;

int gpio_int_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "gpio_int : open\n");

	return 0;
}

int gpio_int_release(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "gpio_int : release\n");

	return 0;
}

ssize_t gpio_int_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	int k_data = 0;
	int ret;

	//printk(KERN_INFO "gpio_int : read\n");

	if( flag_gpioint == 0 )
	{
		if( filp->f_flags & O_NONBLOCK )
			return -EAGAIN;

		//printk(KERN_INFO "gpio_int : waiting gpio int\n");
		ret = wait_event_interruptible(wq_gpioint, flag_gpioint);
	}

	k_data = gpio_get_value(portPin);
	ret = copy_to_user(buf, &k_data, count);
	if( ret < 0 )
		return -1;

	flag_gpioint = 0;
	//printk(KERN_INFO "gpio_int : read done(%d)\n",k_data);

	return count;
}

ssize_t gpio_int_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	int k_data;
	int ret;

	printk(KERN_INFO "gpio_int : write\n");

	ret = copy_from_user(&k_data, buf, count);
	if( ret < 0 )
		return -1;

	//gpio_set_value(portPin, k_data);

	printk(KERN_INFO "gpio_int : write(%d)\n",k_data);

	return count;
}

long gpio_int_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	printk(KERN_INFO "gpio_int : ioctl(cmd:%d)\n",cmd);
	return 0;
}

struct file_operations gpioint_fops = {
	read:		  gpio_int_read,
	write:		  gpio_int_write,
	open:		  gpio_int_open,
	unlocked_ioctl:	gpio_int_ioctl,
	release:	  gpio_int_release
};

irqreturn_t gpioint_handler(int irq, void *data)
{
	//printk(KERN_INFO "gpio_int : in handler(irq:%d)\n",irq);

	flag_gpioint = 1;
	wake_up_interruptible(&wq_gpioint);

	return IRQ_HANDLED;
}

int __init gpio_int_init(void)
{
	int error;

	printk(KERN_INFO "gpio_int : module is up\n");

	if( gpioint_major )
	{
		gpioint_dev = MKDEV(gpioint_major,gpioint_minor);
		error = register_chrdev_region(gpioint_dev, 1, "and_gpio_int");
	}
	else
	{
		error = alloc_chrdev_region(&gpioint_dev, gpioint_minor, 1, "and_gpio_int");
		gpioint_major = MAJOR(gpioint_dev);
	}

	if( error < 0 )
	{
		printk(KERN_WARNING "gpio_int : alloc_major error\n");
		return error;
	}

	printk(KERN_INFO "gpio_int : alloc_major ok => major_num : %d\n",gpioint_major);

	cdev_init(&gpioint_cdev, &gpioint_fops);
	gpioint_cdev.owner = THIS_MODULE;
	error = cdev_add(&gpioint_cdev, gpioint_dev, 1);
	if( error < 0 )
	{
		printk(KERN_WARNING "gpio_int : register_chrdev error\n");
		return error;
	}

	if( gpio_request(portPin, "and_gpio_int") == -EBUSY )
		printk(KERN_WARNING "gpio_int : gpio line is busy\n");

	gpio_direction_input(portPin);
	irq_number = gpio_to_irq(portPin);
	irq_set_irq_type(irq_number, IRQ_TYPE_EDGE_FALLING);
	//irq_set_irq_type(irq_number, IRQ_TYPE_EDGE_BOTH);
	printk(KERN_INFO "gpio_int : irq_number => %d\n",irq_number);

	if( request_irq(irq_number, gpioint_handler, IRQF_SHARED, "and_gpio_int", &gpioint_dev) != 0 )
	{
		printk(KERN_WARNING "gpio_int : request_irq fail\n");
		return -ENOENT;
	}

	printk(KERN_INFO "request_irq ok\n");

	printk(KERN_INFO "gpio_int : module insert done");
	printk(KERN_INFO "mknod /dev/and_gpio_int c %d %d\n",gpioint_major, gpioint_minor);
	printk(KERN_INFO "chmod 666 /dev/and_gpio_int\n");

	return 0;
}

void __exit gpio_int_exit(void)
{
	printk(KERN_INFO "gpio_int : module is down\n");

	free_irq(irq_number, &gpioint_dev);
	gpio_free(portPin);

	cdev_del(&gpioint_cdev);
	unregister_chrdev_region(gpioint_dev, 1);

	printk(KERN_INFO "gpio_int : module delete done\n");
}

module_init(gpio_int_init);
module_exit(gpio_int_exit);

MODULE_LICENSE("Dual BSD/GPL");
