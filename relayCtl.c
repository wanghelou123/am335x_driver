
#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/gpio.h>
#include <linux/spinlock.h>
#include <linux/klha_gpio.h>
#include <asm/uaccess.h> //copy_to_user 头文件
#define	DEVICE_NAME	"relayCtl"
#define pr_fmt(fmt) "relayCtl:" fmt
extern unsigned int device_type;
static int major = 0;
static struct class *relay_ctl_class;

struct relay_ctl_dev {
	struct timer_list	relay_ctl_timer;
	int relay_status;
	int relay_command_status;
	int relay_frequency;
	int relay_id;
	spinlock_t	relay_lock;
};
struct relay_ctl_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct gpio_klha_platform_data *data;
	char * relayStatus;
	struct relay_ctl_dev *relay_dev;
};
//struct relay_ctl_data * relay_ctl;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_relayCtl_lock);


static const struct of_device_id of_relay_ctl_match[] = {
	{.compatible = "relayCtl",},
	{},
};

static void relay_ctl_do_timer(unsigned long arg)
{
	struct relay_ctl_dev *dev = (struct relay_ctl_dev *)arg;

	spin_lock(&(dev->relay_lock));

	//继电器引脚电平翻转
	dev->relay_status = (!(dev->relay_status))&0x01;
	gpio_direction_output(dev->relay_id, dev->relay_status);

	//在定时器函数中重新启动定时器以实现轮询的目的
	dev->relay_ctl_timer.expires = jiffies + HZ/(2*(dev->relay_frequency));
	dev->relay_ctl_timer.function = &relay_ctl_do_timer;
	dev->relay_ctl_timer.data = (unsigned long)dev;
	add_timer(&(dev->relay_ctl_timer));

	spin_unlock(&(dev->relay_lock));
}

static size_t relay_ctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long err = 0;
	int i;
	struct relay_ctl_data* pdata;
	pdata= filp->private_data;
	//printk("pdata_read():count = %d\n.", count);
	//printk("pdata->data->num = %d\n.", pdata->data->nums+(pdata->data->device_status?1:0));

	count = min(count, (size_t)(pdata->data->nums + (pdata->data->device_status?1:0)));

	if(pdata->data->device_status) {//只返回控制盒是否插入的状态
		pdata->relayStatus[0] = (gpio_get_value(pdata->data->device_status))?1:0;
	}
	else {
		for(i=0;i<pdata->data->nums;i++){//返回继电器的状态
			pdata->relayStatus[i] = gpio_get_value(	pdata->data->gpio_array[i]);
		}	
	}

	err = copy_to_user(buf, pdata->relayStatus, count);

	if(err)
		return -EFAULT;
	else
		return  count;
}

static int relay_ctl_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	//pr_info(" %s\n", __func__);
	struct relay_ctl_data * pdata;
	pdata= filp->private_data;

	cmd = _IOC_NR(cmd);
	if(cmd!=0 && cmd !=1 && device_type !=4)	
		return -EINVAL;

	if(device_type !=4 && (arg >= pdata->data->nums || arg < 0))
		return -EINVAL;
	
	if(device_type == 4 && (arg ==7||arg==8)) {
		arg -= 2;
		if(cmd == 0 && pdata->relay_dev[arg].relay_command_status != 0) {
			pdata->relay_dev[arg].relay_command_status = 0;
			del_timer_sync(&(pdata->relay_dev[arg].relay_ctl_timer));
			gpio_direction_output(pdata->data->gpio_array[arg], 1);
		}else if(cmd !=0 && pdata->relay_dev[arg].relay_command_status != 1) {
			 pdata->relay_dev[arg].relay_command_status = 1;
			 pdata->relay_dev[arg].relay_id = pdata->data->gpio_array[arg];
			 pdata->relay_dev[arg].relay_frequency = cmd;
			 init_timer(&(pdata->relay_dev[arg].relay_ctl_timer));
			 pdata->relay_dev[arg].relay_ctl_timer.expires = jiffies + HZ/(2*pdata->relay_dev[arg].relay_frequency);
			 pdata->relay_dev[arg].relay_ctl_timer.function = &relay_ctl_do_timer;
			 pdata->relay_dev[arg].relay_ctl_timer.data = (unsigned long )&pdata->relay_dev[arg];
			 add_timer(&(pdata->relay_dev[arg].relay_ctl_timer));
		}

		return 0;
	}

	switch(cmd)	 {
		case 0:
			gpio_direction_output(pdata->data->gpio_array[arg], 0);	
			pdata->relayStatus[arg+1] = 0;
		break;
		case 1:
			gpio_direction_output(pdata->data->gpio_array[arg], 1);	
			pdata->relayStatus[arg+1] = 1;
		break;
		default:
			return -EINVAL;
	}

	return 0;
}


static int relay_ctl_open(struct inode *inode, struct file *filp)
{
	struct relay_ctl_data * pdata;
	int status = - ENXIO;

	mutex_lock(&device_relayCtl_lock);
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
	mutex_unlock(&device_relayCtl_lock);

	return status;
}

static int relay_ctl_release(struct inode *inode, struct file *filp)
{
	struct relay_ctl_data * pdata;
	pdata= filp->private_data;

	mutex_lock(&device_relayCtl_lock);
	pdata->users--;
	mutex_unlock(&device_relayCtl_lock);

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
		printk(KERN_NOTICE "Error %d adding relayCtl",err);
}

static int __devinit relay_ctl_probe(struct platform_device *pdev)
{
	int		status;
	int i =0;
	struct device *dev;
	struct relay_ctl_data * relay_ctl;

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

	relay_ctl->relay_dev = kzalloc((relay_ctl->data->nums + 1)*sizeof(struct relay_ctl_dev), GFP_KERNEL);
	if(!relay_ctl->relay_dev)
		return -ENOMEM;
	

	//打印管脚信息,并保存管脚状态
	for(i=0; i<relay_ctl->data->nums; i++) {
		gpio_request(relay_ctl->data->gpio_array[i], NULL);
		gpio_direction_output(relay_ctl->data->gpio_array[i], 1);
		relay_ctl->relayStatus[i+1] = (gpio_get_value(relay_ctl->data->gpio_array[i]))? 1: 0;

		pr_info(" relayCtl channel_%d gpio=>%d\n", i, relay_ctl->data->gpio_array[i]);	
	}

	/* 创建一个设备节点，名为/dev/relayCtl */
	dev = device_create(relay_ctl_class, &pdev->dev, relay_ctl->devt, relay_ctl, "relayCtl");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

	if (status == 0) {
		list_add(&relay_ctl->device_entry, &device_list);
	}

	if (status == 0)
		platform_set_drvdata(pdev, relay_ctl);	
	else {
		kfree(relay_ctl->relayStatus);
		kfree(relay_ctl->relay_dev);
		kfree(relay_ctl);
	}

	return status;
}

static int __devexit relay_ctl_remove(struct platform_device *pdev)
{
	struct relay_ctl_data * pdata;
	pdata=  platform_get_drvdata(pdev);

	mutex_lock(&device_relayCtl_lock);
	cdev_del(&pdata->cdev);//在系统中删除 cdev
	device_destroy(relay_ctl_class, pdata->devt);//删除/dev上的节点
	unregister_chrdev_region(pdata->devt, 1);//释放设备号
	if (pdata->users == 0)
		kfree(pdata->relayStatus);
		kfree(pdata->relay_dev);
		kfree(pdata);

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
