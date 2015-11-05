#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/klha_gpio.h>

#define DEVICE_NAME "buzzer"//蜂鸣器

static int major = 0;
static struct class * buzzer_class;
struct buzzer_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct gpio_klha_platform_data	*data;
};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_buzzer_lock);
static const struct of_device_id of_buzzer_match[] = {
	{.compatible = DEVICE_NAME},
	{},
};

static int buzzer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){

	struct buzzer_data		*pdata;
	pdata = filp->private_data;

	switch(cmd){
		case 0:	
			gpio_direction_output(pdata->data->gpio_array[0], 0);
			//printk("set usb to 0\n");
			//printk("get usb state %d\n",gpio_get_value(USB));
			return 0;
			
		case 1:
			gpio_direction_output(pdata->data->gpio_array[0], 1);
			//printk("set usb to 1\n");
			//printk("get usb state %d\n",gpio_get_value(USB));
			return 0;
		default:
			return -EINVAL;			
	} 
}

static int buzzer_open(struct inode *inode, struct file *filp)
{
	struct buzzer_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_buzzer_lock);
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

	mutex_unlock(&device_buzzer_lock);

	return status;
}

static int buzzer_release(struct inode *inode, struct file *filp)
{
	struct buzzer_data		*pdata;
	pdata = filp->private_data;

	mutex_lock(&device_buzzer_lock);
	pdata->users--;
	mutex_unlock(&device_buzzer_lock);

	return 0;
}

static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.unlocked_ioctl =     buzzer_ioctl,
		.open			=	buzzer_open,
		.release		=	buzzer_release,
}; 

static void buzzer_setup_cdev(struct buzzer_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &dev_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding %s\n", err, DEVICE_NAME);
}

static int __devinit buzzer_probe(struct platform_device *pdev)
{
	int status;
	int i=0;
	struct buzzer_data * pdata;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	buzzer_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}

	//打印管脚信息
	for(i = 0; i<pdata->data->nums; i++) {
		gpio_request(pdata->data->gpio_array[i], NULL);
		printk(KERN_NOTICE "%s%d gpio => %d\n", DEVICE_NAME, i, pdata->data->gpio_array[i]);
	}

	/*创建一个设备节点，名为/dev/buzzer */
	struct device *dev;
	dev = device_create(buzzer_class, &pdev->dev, pdata->devt, pdata, DEVICE_NAME);
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;
	
	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}
	
	return status;
}

static int __devexit buzzer_remove(struct platform_device *pdev)
{
	struct buzzer_data *pdata;
	pdata = platform_get_drvdata(pdev);
	cdev_del(&pdata->cdev);
	device_destroy(buzzer_class, pdata->devt);
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

static struct platform_driver buzzer_driver = {
	.probe = buzzer_probe,
	.remove = __devexit_p(buzzer_remove),
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_buzzer_match,
	},
};

static int __init dev_init(void)
{
	int status;

	buzzer_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(buzzer_class)) {
			return PTR_ERR(buzzer_class);
	}
	status = platform_driver_register(&buzzer_driver);

	if(status < 0 ) {
		printk(KERN_ERR "register %s to platform failed!\n", DEVICE_NAME);
		class_destroy(buzzer_class);
	}else
		printk(KERN_NOTICE "%s driver register success.\n", DEVICE_NAME);

	return status;
}

static void __exit dev_exit(void)
{
	platform_driver_unregister(&buzzer_driver);
	class_destroy(buzzer_class);
	printk(KERN_NOTICE "%s driver unregister success.\n", DEVICE_NAME);
}
module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WHL");
