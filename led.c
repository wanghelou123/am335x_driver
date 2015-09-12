#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/fs.h>  
#include <asm/gpio.h>

#define DEVICE_NAME "leds"
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

static unsigned long led_table[] = {
	GPIO_TO_PIN(2, 12),  //gpio2_12, led zigbee
	GPIO_TO_PIN(2, 13),  //gpio2_13, led net
	GPIO_TO_PIN(2, 14),  //gpio2_14, led usb
//	GPIO_TO_PIN(2, 10),  //gpio2_10, led sys,转接转上的led
};


static int am3352_leds_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long arg){
	switch(cmd){
		case 0:	
			gpio_direction_output(led_table[arg], 0);	
			//printk("gpio_set_value, port=%d, cmd=%d\n", led_table[arg], cmd);
			//printk("==>gpio_get_value, port=%d, states=%d\n", led_table[arg], gpio_get_value(led_table[arg]));
			return 0;
			
		case 1:
			gpio_direction_output(led_table[arg], 1);
			//printk("gpio_set_value, port=%d, cmd=%d\n", led_table[arg], cmd);
			//printk("==>gpio_get_value, port=%d, states=%d\n", led_table[arg], gpio_get_value(led_table[arg]));
			return 0;
		default:
			return -EINVAL;			
	} 
}

static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.unlocked_ioctl =     am3352_leds_ioctl,
}; 

static struct miscdevice misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = DEVICE_NAME,
		.fops  = &dev_fops,
};

static int __init dev_init(void){
	int ret;
	int ret2=0;
	/*
	ret2 = gpio_request(GPIO_TO_PIN(2, 10), "gpio2_10");
	if(ret2!=0) {
		printk("gpio_request port(2, 10) error %d.\n", ret2);
	}*/
	ret2 = gpio_request(GPIO_TO_PIN(2, 12), "gpio2_12");
	if(ret2!=0) {
		printk("gpio_request port(2, 12) error %d.\n", ret2);
	}
	ret2 = gpio_request(GPIO_TO_PIN(2, 13), "gpio2_13");
	if(ret2!=0) {
		printk("gpio_request port(2, 13) error %d.\n", ret2);
	}
	ret2 = gpio_request(GPIO_TO_PIN(2, 14), "gpio2_14");
	if(ret2!=0) {
		printk("gpio_request port(2, 14) error %d.\n", ret2);
	}

	ret = misc_register(&misc);
	if(ret < 0) {
		printk(DEVICE_NAME ":can't register device.");
	}

	printk(DEVICE_NAME"\tinitialized\n");
	
	return ret;
}

static void __exit dev_exit(void){
	//gpio_free(GPIO_TO_PIN(2, 10));
	gpio_free(GPIO_TO_PIN(2, 12));
	gpio_free(GPIO_TO_PIN(2, 13));
	gpio_free(GPIO_TO_PIN(2, 14));
	//gpio_direction_output(GPIO_TO_PIN(2, 10),0);
	//gpio_direction_output(GPIO_TO_PIN(0, 17),0);
	//gpio_direction_output(GPIO_TO_PIN(1, 8),0);

	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WangHeLou");
