#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/fs.h>  
#include <asm/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/klha_gpio.h>
#include <asm/uaccess.h> //copy_to_user 头文件
#define DEVICE_NAME "button"

static volatile char key_value = 0;
static volatile int ev_press = 0;
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

static int major = 0;
static struct class * button_class;
struct button_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct gpio_klha_platform_data	*data;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_button_lock);

static const struct of_device_id of_button_match[] = {
	{.compatible = "button",},
	{},
};
/*描述按键的结构体*/
struct button_irq_desc {
	int irq;
	int gpio;
	char *name;
};

struct button_irq_desc mybutton;

static irqreturn_t button_handler(int irq, void *dev_id) 
{
	key_value = (~gpio_get_value(mybutton.gpio)) & 0x01;
	ev_press = 1;

	wake_up_interruptible(&button_waitq);

	return IRQ_RETVAL(IRQ_HANDLED);

}

static int button_read(struct file *filp, char __user *buffer, size_t count, loff_t *offp)
{
	unsigned long err;
	if(!ev_press) {
		if(filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(button_waitq, ev_press);
	} 

	err = copy_to_user(buffer, (void *)&key_value, min(sizeof(key_value), count));

	//key_value = 0;
	ev_press = 0;

	if(err)
		return EFAULT;
	else 
		return min(sizeof(key_value), count);
	
}

 

static int button_open(struct inode *inode, struct file *filp)
{
	struct button_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_button_lock);
	list_for_each_entry(pdata, &device_list, device_entry) {
		if(pdata->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if(status == 0) {
		pdata->users++;
		filp->private_data = pdata;
	}

	mutex_unlock(&device_button_lock);

	return status;
}

static int button_release(struct inode *inode, struct file *filp)
{
	struct button_data	*pdata;
	pdata = filp->private_data;

	mutex_lock(&device_button_lock);
	pdata->users--;
	mutex_unlock(&device_button_lock);

	return 0;
}
	


static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.open	= button_open,
		.release	= button_release,	
		.read	= button_read,
		//.poll	= button_poll,
};
static void button_setup_cdev(struct button_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &dev_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding button\n", err);
}

static int __devinit button_probe(struct platform_device *pdev)
{
	int status;
	int ret;
	unsigned long irqflags;
	struct button_data * pdata;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	button_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}

	//打印管脚信息
	int i=0;
	for(i = 0; i<pdata->data->nums; i++) {
		printk(KERN_NOTICE "button%d gpio=>%d\n", i, pdata->data->gpio_array[i]);
	}

	/*创建一个设备节点，名为/dev/button */
	struct device *dev;
	dev = device_create(button_class, &pdev->dev, pdata->devt, pdata, "button");
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;
	
	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}


	mybutton.irq = gpio_to_irq( pdata->data->gpio_array[0]),	
	mybutton.gpio = pdata->data->gpio_array[0];
	mybutton.name="KEY";
	printk("the gpio_key irq is [%d].\n", mybutton.irq);

	ret = gpio_request( pdata->data->gpio_array[0], "reset");
	if(ret<0)
		printk("gpio_request error, %d\n",  pdata->data->gpio_array[0]);
	
	irqflags = IRQ_TYPE_EDGE_BOTH;

	ret = request_irq(mybutton.irq, button_handler, irqflags, mybutton.name,NULL );
	if(ret) {
		printk("can't get gpio irq\n");	
		return -1;
	}

	return status;
}

static int __devexit button_remove(struct platform_device *pdev)
{
	struct button_data *pdata;
	pdata = platform_get_drvdata(pdev);
	
	disable_irq(mybutton.irq);
	free_irq(mybutton.irq, NULL);

	cdev_del(&pdata->cdev);
	device_destroy(button_class, pdata->devt);
	unregister_chrdev_region(pdata->devt, 1);
	printk("pdata->users=>%d\n", pdata->users);
	if(pdata->users == 0) {
		int i;
		for(i=0; i<pdata->data->nums ; i++) {
			gpio_free(pdata->data->gpio_array[i]);
		}
		kfree(pdata);
	}
}

static struct platform_driver button_driver = {
	.probe = button_probe,
	.remove = __devexit_p(button_remove),
	.driver = {
		.name = "button",
		.owner = THIS_MODULE,
		.of_match_table = of_button_match,
	},
};

static int __init dev_init(void)
{
	int status;

	button_class = class_create(THIS_MODULE, "button");
	if (IS_ERR(button_class)) {
		return PTR_ERR(button_class);
	}
	status = platform_driver_register(&button_driver);

	if(status < 0 ) {
		printk(KERN_ERR "register button to platform failed!\n");
		class_destroy(button_class);
	}else
		printk(KERN_NOTICE "button driver register success.\n");


	return status;
}

static void __exit dev_exit(void)
{

	platform_driver_unregister(&button_driver);
	class_destroy(button_class);
	printk(KERN_NOTICE "button driver unregister success.\n");
}
module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WangHeLou");
