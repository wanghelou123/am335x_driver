#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

#define DEVICE_NAME "powerRelay"//继电器

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#define	USB			GPIO_TO_PIN(0, 9)
static unsigned long powerRelay[] = {
	//S3C2410_GPD11, //M2M
	//S3C2410_GPD12, //zigbee
	USB, //USB 3G
};


static int powerRelay_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long arg){
	switch(cmd){
		case 0:	
			//s3c2410_gpio_setpin(powerRelay[arg], 0);	
			gpio_direction_output(USB, 0);
			//printk("set usb to 0\n");
			//printk("get usb state %d\n",gpio_get_value(USB));
			return 0;
			
		case 1:
			gpio_direction_output(USB, 1);
			//printk("set usb to 1\n");
			//printk("get usb state %d\n",gpio_get_value(USB));
			return 0;
		default:
			return -EINVAL;			
	} 
}

static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.unlocked_ioctl =     powerRelay_ioctl,
}; 

static struct miscdevice misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = DEVICE_NAME,
		.fops  = &dev_fops,
};

static int __init dev_init(void){
	int ret = 0;
	ret |= gpio_request(USB, NULL);
	if(ret !=0 ) {
		printk("gpio_request error. ret = %d\n", ret);	
	}

	ret = misc_register(&misc);
	if (ret < 0) {
		printk(DEVICE_NAME ":can't register device.");
		return ret;
	}
		
	printk(DEVICE_NAME"\tinitialized\n");
	
	return ret;
}

static void __exit dev_exit(void){
	gpio_free(USB);
	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WHL");
