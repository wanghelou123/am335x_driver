
#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/gpio.h>
#include <linux/klha_gpio.h>
#include <asm/uaccess.h> //copy_to_user 头文件
#define	DEVICE_NAME	"relayCtl"
static int major = 0;
static unsigned long minor = 0;
static struct classs *relay_ctl_class;

struct relay_ctl_data {
	struct cdev			cdev;
	dev_t				devt;
	unsigned			users;
	struct gpio_klha_platform_data *data;
	char * relayStatus;
};
struct relay_ctl_data * relay_ctl;

static DEFINE_MUTEX(device_relayCtl_lock);


static const struct of_device_id of_relay_ctl_match[] = {
	{.compatible = "relayCtl",},
	{},
};

static size_t relay_ctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long err = 0;
	//printk("relay_ctl_read():count = %d\n.", count);
	//printk("relay_ctl->data->num = %d\n.", relay_ctl->data->nums+(relay_ctl->data->device_status?1:0));

	count = min(count, relay_ctl->data->nums + (relay_ctl->data->device_status?1:0));

	relay_ctl->relayStatus[0] = (gpio_get_value(relay_ctl->data->device_status))?1:0;

	err = copy_to_user(buf, relay_ctl->relayStatus, count);

	if(err)
		return -EFAULT;
	else
		return  count;
}

static int relay_ctl_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	//printk("cmd = %d\n", cmd);
	//printk("arg = %d\n", arg);
	if(arg >= relay_ctl->data->nums || arg < 0)
		return -EINVAL;

	switch(cmd)	 {
		case 0:
			gpio_direction_output(relay_ctl->data->gpio_array[arg], 0);	
			relay_ctl->relayStatus[arg+1] = 0;
		break;
		case 1:
			gpio_direction_output(relay_ctl->data->gpio_array[arg], 1);	
			relay_ctl->relayStatus[arg+1] = 1;
		break;
		default:
			return -EINVAL;
	}

	return 0;
}


static int relay_ctl_open(struct inode *inode, struct file *filp)
{
	//struct relay_ctl_data *relay_ctl;	
	//relay_ctl = filp->private_data;
	#if 0
	if(NULL == relay_ctl) {
		printk(KERN_ERR "privte_data is NULL;\n");
		return -ENODEV;
	}
	mutex_lock(&device_relayCtl_lock);
	relay_ctl->users++;
	mutex_unlock(&device_relayCtl_lock);

	#endif
	return 0;

}

static int relay_ctl_release(struct inode *inode, struct file *filp)
{
#if 0
	mutex_lock(&device_relayCtl_lock);
	relay_ctl->users--;
	mutex_unlock(&device_relayCtl_lock);
#endif

	return 0;
}

static const struct file_operations relay_ctl_fops = {
	.owner	=	THIS_MODULE,
	.read	=	relay_ctl_read,
	.unlocked_ioctl = relay_ctl_ioctl,
	.open	=	relay_ctl_open,
	.release	=	relay_ctl_release,
};

static void relay_ctl_setup_cdev(struct relay_ctl_data *relay_ctl, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&relay_ctl->cdev, &relay_ctl_fops);

	err = cdev_add(&relay_ctl->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %s adding relayCtl",err);
}

static int __devinit relay_ctl_probe(struct platform_device *pdev)
{
	int		status;

	relay_ctl = kzalloc(sizeof(*relay_ctl), GFP_KERNEL);
	if(!relay_ctl)
		return -ENOMEM;
		

	status = alloc_chrdev_region(&relay_ctl->devt, 0, 1, DEVICE_NAME);
	if(status < 0 )
		return status;
	major = MAJOR(relay_ctl->devt);

	relay_ctl_setup_cdev(relay_ctl, 0);


	relay_ctl->data = pdev->dev.platform_data;
	if(!relay_ctl->data) {
		printk(KERN_ERR "NOT FIND GPIO");
		return -ENODEV;
	}

	relay_ctl->relayStatus = kzalloc(relay_ctl->data->nums + 1, GFP_KERNEL);
	if(!relay_ctl->relayStatus)
		return -ENOMEM;

	//打印管脚信息,并保存管脚状态
	int i =0;
	for(i=0; i<relay_ctl->data->nums; i++) {
		relay_ctl->relayStatus[i+1] = (gpio_get_value(relay_ctl->data->gpio_array[i]))? 1: 0;
		printk(KERN_NOTICE "relayCtl channel_%d gpio=>%d\n", i, relay_ctl->data->gpio_array[i]);	
	}

	/* 创建一个设备节点，名为/dev/relayCtl */
	struct device *dev;
	dev = device_create(relay_ctl_class, &pdev->dev, relay_ctl->devt, relay_ctl, "relayCtl");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

	if (status == 0)
		platform_set_drvdata(pdev, relay_ctl);	
	else {
		kfree(relay_ctl->relayStatus);
		kfree(relay_ctl);
	}

	return status;
}

static int __devexit relay_ctl_remove(struct platform_device *pdev)
{
	cdev_del(&relay_ctl->cdev);//在系统中删除 cdev
	device_destroy(relay_ctl_class, relay_ctl->devt);//删除/dev上的节点
	unregister_chrdev_region(relay_ctl->devt, 1);//释放设备号
	if (relay_ctl->users == 0)
		kfree(relay_ctl->relayStatus);
		kfree(relay_ctl);

	return 0;
}
static struct platform_driver relay_ctl_driver = {
	.probe	= relay_ctl_probe,
	.remove	= __devexit_p(relay_ctl_remove),
	.driver	= {
		.name	=	"relayCtl",
		.owner	=	THIS_MODULE,
		.of_match_table	=	of_relay_ctl_match,
	},
};
static int __init relay_ctl_init(void)
{
	int status;

	relay_ctl_class = class_create(THIS_MODULE, "relayCtl");
	if (IS_ERR(relay_ctl_class)) {
		return PTR_ERR(relay_ctl_class);
	}
	status = platform_driver_register(&relay_ctl_driver);

	if (status < 0 ) {
		class_destroy(relay_ctl_class);
	}
	else
		printk(KERN_NOTICE "relayCtl driver register success.\n");

	return status;
}
static void __exit relay_ctl_exit(void)
{
	platform_driver_unregister(&relay_ctl_driver);
	class_destroy(relay_ctl_class);
	printk(KERN_NOTICE "relayCtl driver unregister success.\n");
}

module_init(relay_ctl_init);
module_exit(relay_ctl_exit);
MODULE_AUTHOR("He Lou Wang");
MODULE_DESCRIPTION("collector the gpio status.");
MODULE_LICENSE("GPL");
