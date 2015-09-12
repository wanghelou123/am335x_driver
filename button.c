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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/poll.h>
 #include <linux/sched.h>

#define DEVICE_NAME "button"
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#define	RESET_BUTTON	GPIO_TO_PIN(0, 17)

static volatile char key_value = 0;
static volatile int ev_press = 0;
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);


/*描述按键的结构体*/
struct button_irq_desc {
	int irq;
	char *name;
};

struct button_irq_desc mybutton;

static irqreturn_t button_handler(int irq, void *dev_id) 
{
	key_value = gpio_get_value(RESET_BUTTON) & 0x01;
	ev_press = 1;

	wake_up_interruptible(&button_waitq);

	return IRQ_RETVAL(IRQ_HANDLED);

}

static int am3352_button_open(struct inode * inode, struct file *file)
{
	return 0;
}

static int am3352_button_close(struct inode *node, struct file *file)
{

	return 0;
}
static int am3352_button_read(struct file *filp, char __user *buffer, size_t count, loff_t *offp)
{
	unsigned long err;
	if(!ev_press) {
		if(filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(button_waitq, ev_press);
	} 

	err = copy_to_user(buffer, (void *)&key_value, min(sizeof(key_value), count));

	key_value = 0;
	ev_press = 0;

	if(err)
		return EFAULT;
	else 
		return min(sizeof(key_value), count);
	
}

static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.open	= am3352_button_open,
		.release	= am3352_button_close,	
		.read	= am3352_button_read,
		//.poll	= am3352_button_poll,
}; 

static struct miscdevice misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = DEVICE_NAME,
		.fops  = &dev_fops,
};

static int __init dev_init(void){

	int ret;
	unsigned long irqflags;

	ret = misc_register(&misc);
	if(ret < 0) {
		printk(DEVICE_NAME ":can't register device.");
	}

	mybutton.irq = gpio_to_irq(RESET_BUTTON),	
	mybutton.name="KEY";
	printk("the gpio_key irq is [%d].\n", mybutton.irq);

	ret = gpio_request(RESET_BUTTON, "reset");
	if(ret<0)
		printk("gpio_request error, %d\n", RESET_BUTTON);
	
	irqflags = IRQ_TYPE_EDGE_BOTH;

	ret = request_irq(mybutton.irq, button_handler, irqflags, mybutton.name,NULL );
	if(ret) {
		printk("can't get gpio irq\n");	
		return -1;
	}

	printk(DEVICE_NAME"\tinitialized\n");
	
	return ret;
}

static void __exit dev_exit(void){
	disable_irq(mybutton.irq);
	free_irq(mybutton.irq, NULL);
	gpio_free(RESET_BUTTON);

	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WangHeLou");
