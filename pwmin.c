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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/klha_gpio.h>

#define DEVICE_NAME "pwmin"//PWM输入
#define pr_fmt(fmt) "pwmin:" fmt

static int current_channel = 0;
static int major = 0;
static struct class * pwmin_class;
struct pwmin_dev {
	int irq;
	int	freq;
	volatile int count;
};
struct pwmin_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct gpio_klha_platform_data	*data;
	struct timer_list	pwmin_timer;
	struct pwmin_dev		*pwmin_dev;
};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_pwmin_lock);
static const struct of_device_id of_pwmin_match[] = {
	{.compatible = DEVICE_NAME},
	{},
};

static void pwmin_do_timer(unsigned long arg)
{
	//pr_info("%s\n", __func__);
	struct pwmin_data* pdata = (struct pwmin_data*)arg;
	int i=0;

	for(i = 0; i<pdata->data->nums; i++) {
		pdata->pwmin_dev[i].freq = pdata->pwmin_dev[i].count*2;
		pdata->pwmin_dev[i].count = 0;
	}

	pdata->pwmin_timer.expires = jiffies+HZ/2;
	add_timer(&(pdata->pwmin_timer));

	return IRQ_RETVAL(IRQ_HANDLED);
}

static irqreturn_t pwmin_irq_handler(int irq, void *dev_id)
{
	struct pwmin_dev * dev = (struct pwmin_dev *)dev_id;
	dev->count++;	
	return IRQ_RETVAL(IRQ_HANDLED);
}

static size_t pwmin_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long err=0;
	int i;
	struct pwmin_data *pdata;
	pdata = filp->private_data;
	count = min(count, 4);

	err = copy_to_user(buf, &pdata->pwmin_dev[current_channel].freq, count);

	if(err)
		return -EFAULT;
	else
		return count;
}

static int pwmin_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){

	struct pwmin_data		*pdata;
	pdata = filp->private_data;

	cmd = _IOC_NR(cmd);
	
	if(cmd <0 || cmd >= pdata->data->nums)	
		return -EINVAL;	

	//pr_info("%s:cmd:%d\n", __func__, cmd);
	current_channel = cmd;

	return 0;
}

static int pwmin_open(struct inode *inode, struct file *filp)
{
	struct pwmin_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_pwmin_lock);
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

	mutex_unlock(&device_pwmin_lock);

	return status;
}

static int pwmin_release(struct inode *inode, struct file *filp)
{
	struct pwmin_data		*pdata;
	pdata = filp->private_data;

	mutex_lock(&device_pwmin_lock);
	pdata->users--;
	mutex_unlock(&device_pwmin_lock);

	return 0;
}

static struct file_operations dev_fops = {
		.owner =     THIS_MODULE,
		.unlocked_ioctl =     pwmin_ioctl,
		.open			=	pwmin_open,
		.release		=	pwmin_release,
		.read			=	pwmin_read,
}; 

static void pwmin_setup_cdev(struct pwmin_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &dev_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding %s\n", err, DEVICE_NAME);
}

static int __devinit pwmin_probe(struct platform_device *pdev)
{
	int status;
	int i=0;
	struct pwmin_data * pdata;
	int ret;
	unsigned long irqflags;
	//pr_info("%s:%d\n", __func__, __LINE__);
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	pwmin_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}

	pdata->pwmin_dev = kzalloc((pdata->data->nums)*sizeof(struct pwmin_dev), GFP_KERNEL);
	if(!pdata->pwmin_dev)
		return -ENOMEM;

	//打印管脚信息
	for(i = 0; i<pdata->data->nums; i++) {
		gpio_request(pdata->data->gpio_array[i], NULL);
		pr_info("%s%d gpio => %d\n", DEVICE_NAME, i, pdata->data->gpio_array[i]);
	}

	/*创建一个设备节点，名为/dev/pwmin */
	struct device *dev;
	dev = device_create(pwmin_class, &pdev->dev, pdata->devt, pdata, DEVICE_NAME);
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;
	
	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}

	//添加定时器
	init_timer(&(pdata->pwmin_timer));
	pdata->pwmin_timer.expires = jiffies+HZ*2;
	pdata->pwmin_timer.function = &pwmin_do_timer;
	pdata->pwmin_timer.data = (unsigned long)pdata;
	add_timer(&(pdata->pwmin_timer));

	//申请中断
	for(i = 0; i<pdata->data->nums; i++) {
		pdata->pwmin_dev[i].irq = gpio_to_irq(pdata->data->gpio_array[i]);	
		//irqflags=IRQ_TYPE_EDGE_BOTH;
		irqflags=IRQF_TRIGGER_FALLING;
		ret = request_irq(pdata->pwmin_dev[i].irq, pwmin_irq_handler,irqflags, "pwmin", &pdata->pwmin_dev[i]);
		if(ret) {
			pr_err("cat't get gpio irq\n");
		}
	}
	
	return status;
}

static int __devexit pwmin_remove(struct platform_device *pdev)
{
	struct pwmin_data *pdata;
	pdata = platform_get_drvdata(pdev);
	cdev_del(&pdata->cdev);
	device_destroy(pwmin_class, pdata->devt);
	unregister_chrdev_region(pdata->devt, 1);
	pr_info("pdata->users=>%d\n", pdata->users);
	if(pdata->users == 0) {
		int i;
		//取消时器
		del_timer_sync(&(pdata->pwmin_timer));

		for(i=0; i<pdata->data->nums ; i++) {
			disable_irq(pdata->pwmin_dev[i].irq);
			free_irq(pdata->pwmin_dev[i].irq, &(pdata->pwmin_dev[i]));//释放irq
			gpio_free(pdata->data->gpio_array[i]);//释放GPIO
		}
		kfree(pdata->pwmin_dev);
		kfree(pdata);
	}
}

static struct platform_driver pwmin_driver = {
	.probe = pwmin_probe,
	.remove = __devexit_p(pwmin_remove),
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_pwmin_match,
	},
};

static int __init dev_init(void)
{
	int status;

	pwmin_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(pwmin_class)) {
			return PTR_ERR(pwmin_class);
	}
	status = platform_driver_register(&pwmin_driver);

	if(status < 0 ) {
		printk(KERN_ERR "register %s to platform failed!\n", DEVICE_NAME);
		class_destroy(pwmin_class);
	}else
		printk(KERN_NOTICE "%s driver register success.\n", DEVICE_NAME);

	return status;
}

static void __exit dev_exit(void)
{
	platform_driver_unregister(&pwmin_driver);
	class_destroy(pwmin_class);
	printk(KERN_NOTICE "%s driver unregister success.\n", DEVICE_NAME);
}
module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WHL");
