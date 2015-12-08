
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
#define	DEVICE_NAME	"ioGather"
static int major = 0;
static struct class *io_gather_class;

struct io_gather_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct gpio_klha_platform_data *data;
	char			*channel_value;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_ioGather_lock);


static const struct of_device_id of_io_gather_match[] = {
	{.compatible = "ioGather",},
	{},
};

static size_t io_gather_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	unsigned long err = 0;
	int i = 0;
	struct io_gather_data *io_gather;	
	io_gather = filp->private_data;
	
	count = min(count, io_gather->data->nums);

	for(i=0; i<io_gather->data->nums; i++) {
		//printk("[%d]=%d==>%d\n", i,io_gather->data->gpio_array[i], gpio_get_value(io_gather->data->gpio_array[i]) );
		if(gpio_get_value(io_gather->data->gpio_array[i])) {
			io_gather->channel_value[i] = (io_gather->data->active_low)? 0 : 1;
		}else {
			io_gather->channel_value[i] = (io_gather->data->active_low)? 1 : 0;
		}
	}

	err = copy_to_user(buf, (void *)io_gather->channel_value, count);

	if(err)
		return -EFAULT;
	else
		return count;
}

static int io_gather_open(struct inode *inode, struct file *filp)
{
	struct io_gather_data *io_gather;	
	int status = -ENXIO;

	mutex_lock(&device_ioGather_lock);
	list_for_each_entry(io_gather, &device_list, device_entry) {
		if(io_gather->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if(status == 0) {
		io_gather->users++;
		filp->private_data= io_gather ;
		
	}

	mutex_unlock(&device_ioGather_lock);

	return status;
}

static int io_gather_release(struct inode *inode, struct file *filp)
{
	struct io_gather_data *io_gather;	
	io_gather = filp->private_data;

	mutex_lock(&device_ioGather_lock);
	io_gather->users--;
	mutex_unlock(&device_ioGather_lock);

	return 0;
}

static const struct file_operations io_gather_fops = {
	.owner	=	THIS_MODULE,
	.read	=	io_gather_read,
	.open	=	io_gather_open,
	.release	=	io_gather_release,
};

static void io_gather_setup_cdev(struct io_gather_data *io_gather, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&io_gather->cdev, &io_gather_fops);

	err = cdev_add(&io_gather->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding ioGather",err);
}

static int __devinit io_gather_probe(struct platform_device *pdev)
{
	int		status;
	struct io_gather_data * io_gather;

	io_gather = kzalloc(sizeof(*io_gather), GFP_KERNEL);
	if(!io_gather)
		return -ENOMEM;
		

	status = alloc_chrdev_region(&io_gather->devt, 0, 1, DEVICE_NAME);
	if(status < 0 )
		return status;
	major = MAJOR(io_gather->devt);

	io_gather_setup_cdev(io_gather, 0);


	io_gather->data = pdev->dev.platform_data;
	if(!io_gather->data) {
		printk(KERN_ERR "NOT FIND GPIO");
		return -ENODEV;
	}

	io_gather->channel_value = kzalloc(io_gather->data->nums, GFP_KERNEL);
	if(!io_gather->channel_value)
		return -ENOMEM;

	//打印管脚信息
	int i =0;
	for(i=0; i<io_gather->data->nums; i++) {
		gpio_request(io_gather->data->gpio_array[i], NULL);
		printk(KERN_NOTICE "ioGather channel_%d gpio=>%d\n", i, io_gather->data->gpio_array[i]);	
	}

	/* 创建一个设备节点，名为/dev/ioGather */
	struct device *dev;
	dev = device_create(io_gather_class, &pdev->dev, io_gather->devt, io_gather, "ioGather");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	
	if (status == 0) {
		list_add(&io_gather->device_entry, &device_list);
	}

	if (status == 0)
		platform_set_drvdata(pdev, io_gather);	
	else {
		kfree(io_gather->channel_value);
		kfree(io_gather);
	}

	return status;
}

static int __devexit io_gather_remove(struct platform_device *pdev)
{
	int i;
	struct io_gather_data *io_gather;	
	io_gather = platform_get_drvdata(pdev);
	cdev_del(&io_gather->cdev);//在系统中删除 cdev
	device_destroy(io_gather_class, io_gather->devt);//删除/dev上的节点
	unregister_chrdev_region(io_gather->devt, 1);//释放设备号
	if (io_gather->users == 0){
		for(i=0; i<io_gather->data->nums; i++) {
			gpio_free(io_gather->data->gpio_array[i]);
		}
		kfree(io_gather->channel_value);
		kfree(io_gather);
	}

	return 0;
}
static struct platform_driver io_gather_driver = {
	.probe	= io_gather_probe,
	.remove	= __devexit_p(io_gather_remove),
	.driver	= {
		.name	=	"ioGather",
		.owner	=	THIS_MODULE,
		.of_match_table	=	of_io_gather_match,
	},
};
static int __init io_gather_init(void)
{
	int status;

	io_gather_class = class_create(THIS_MODULE, "ioGather");
	if (IS_ERR(io_gather_class)) {
		return PTR_ERR(io_gather_class);
	}
	status = platform_driver_register(&io_gather_driver);

	if (status < 0 ) {
		printk(KERN_ERR "register ioGather to platform failed!\n");	
		class_destroy(io_gather_class);
	}
	else
		printk(KERN_NOTICE "ioGather driver register success.\n");

	return status;
}
static void __exit io_gather_exit(void)
{
	platform_driver_unregister(&io_gather_driver);
	class_destroy(io_gather_class);
	printk(KERN_NOTICE "ioGather driver unregister success.\n");
}

module_init(io_gather_init);
module_exit(io_gather_exit);
MODULE_AUTHOR("He Lou Wang");
MODULE_DESCRIPTION("collector the gpio status.");
MODULE_LICENSE("GPL");
