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
#define DEVICE_NAME "leds"

static int major = 0;
static struct class * myleds_class;
struct leds_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct gpio_klha_platform_data	*data;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_leds_lock);

static const struct of_device_id of_myleds_match[] = {
	{.compatible = "myleds",},
	{},
};

static int myleds_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct leds_data	*pdata;
	pdata = filp->private_data;

	switch(cmd){
		case 0:	
			gpio_direction_output(pdata->data->gpio_array[arg], 0);	
			//printk("gpio_set_value, port=%d, cmd=%d\n", led_table[arg], cmd);
			//printk("==>gpio_get_value, port=%d, states=%d\n", led_table[arg], gpio_get_value(led_table[arg]));
			return 0;
			
		case 1:
			gpio_direction_output(pdata->data->gpio_array[arg], 1);	
			//printk("gpio_set_value, port=%d, cmd=%d\n", led_table[arg], cmd);
			//printk("==>gpio_get_value, port=%d, states=%d\n", led_table[arg], gpio_get_value(led_table[arg]));
			return 0;
		default:
			return -EINVAL;			
	} 
}

static int myleds_open(struct inode *inode, struct file *filp)
{
	struct leds_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_leds_lock);
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

	mutex_unlock(&device_leds_lock);

	return status;
}

static int myleds_release(struct inode *inode, struct file *filp)
{
	struct leds_data	*pdata;
	pdata = filp->private_data;

	mutex_lock(&device_leds_lock);
	pdata->users--;
	mutex_unlock(&device_leds_lock);

	return 0;
}
	

static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.unlocked_ioctl =     myleds_ioctl,
		.open	=	myleds_open,
		.release	= myleds_release,
}; 

static void myleds_setup_cdev(struct leds_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &dev_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding leds\n", err);
}

static int __devinit myleds_probe(struct platform_device *pdev)
{
	int status;
	struct leds_data * pdata;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	myleds_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}

	//打印管脚信息
	int i=0;
	for(i = 0; i<pdata->data->nums; i++) {
		gpio_request(pdata->data->gpio_array[i], NULL);
		printk(KERN_NOTICE "led%d gpio=>%d\n", i, pdata->data->gpio_array[i]);
	}

	/*创建一个设备节点，名为/dev/leds */
	struct device *dev;
	dev = device_create(myleds_class, &pdev->dev, pdata->devt, pdata, "leds");
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;
	
	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}
	
	return status;
}

static int __devexit myleds_remove(struct platform_device *pdev)
{
	struct leds_data *pdata;
	pdata = platform_get_drvdata(pdev);
	cdev_del(&pdata->cdev);
	device_destroy(myleds_class, pdata->devt);
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

static struct platform_driver myleds_driver = {
	.probe = myleds_probe,
	.remove = __devexit_p(myleds_remove),
	.driver = {
		.name = "myleds",
		.owner = THIS_MODULE,
		.of_match_table = of_myleds_match,
	},
};

static int __init dev_init(void)
{
	int status;

	myleds_class = class_create(THIS_MODULE, "myleds");
	if (IS_ERR(myleds_class)) {
			return PTR_ERR(myleds_class);
	}
	status = platform_driver_register(&myleds_driver);

	if(status < 0 ) {
		printk(KERN_ERR "register myleds to platform failed!\n");
		class_destroy(myleds_class);
	}else
		printk(KERN_NOTICE "myleds driver register success.\n");

	return status;
}

static void __exit dev_exit(void)
{
	platform_driver_unregister(&myleds_driver);
	class_destroy(myleds_class);
	printk(KERN_NOTICE "myleds driver unregister success.\n");
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_DESCRIPTION("led driver.");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WangHeLou");
