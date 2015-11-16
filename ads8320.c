
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h> //copy_to_user 头文件
#include <linux/cdev.h>
#include <asm/gpio.h>
#include <linux/klha_cd4051.h>

#define ADS8320_MAJOR			154	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static	int ads8320_major = ADS8320_MAJOR;
unsigned long		minor = 0;

static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct ads8320_data {
	struct cdev			cdev;
	dev_t				devt;
	spinlock_t			spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct mutex		buf_lock;
	unsigned			users;
	u8					*buffer;
	struct cd4051_platform_data *data;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static void ads8320_complete(void *arg)
{
	complete(arg);//唤醒完成量
}

	static ssize_t
ads8320_sync(struct ads8320_data *ads8320, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);//声明并且初始化一个"完成量"
	int status;

	message->complete = ads8320_complete; //传输完成后的回调函数
	message->context = &done;//回调函数的参数

	spin_lock_irq(&ads8320->spi_lock);
	if (ads8320->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(ads8320->spi, message);
	spin_unlock_irq(&ads8320->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);//等待一个完成量被唤醒
		status = message->status;
		if (status == 0)
			status = message->actual_length; //传输完成的位数
	}

	return status;
}
	static inline ssize_t
ads8320_sync_read(struct ads8320_data *ads8320, size_t len)
{
	struct spi_transfer t = {
		.rx_buf = ads8320->buffer,
		.len	= 3, /* 片选后4.5 ~ 5 个周期用于采样, 16 bit为数据，总共需要3个字节, 一个时钟周期是416ns */
	};

	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return ads8320_sync(ads8320, &m);
}

	static ssize_t
ads8320_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct ads8320_data	*ads8320;
	ssize_t	status = 0;

	ads8320 = filp->private_data;
	mutex_lock(&ads8320->buf_lock);


	memset(ads8320->buffer, 0, 3);
	status = ads8320_sync_read(ads8320, count);
	if(status > 0 ) {
		unsigned long missing;
		int value; 
		value = ads8320->buffer[0]<<16 | ads8320->buffer[1]<<8 | ads8320->buffer[2];
		value = (value>>3)&0xFFFF;
		//printk("value1 = %x\n", value);

		missing = copy_to_user(buf, &value,  4);
		status = 4;
		if(missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&ads8320->buf_lock);

	//	printk("buffer[0]%2x\n", ads8320->buffer[0]);
	//	printk("buffer[1]%2x\n", ads8320->buffer[1]);
	//	printk("buffer[2]%2x\n", ads8320->buffer[2]);

	return status;
}

	static long
ads8320_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ads8320_data	*ads8320;
	ads8320 = filp->private_data;

	if(arg<0||arg>8)
		return -1;

	//printk("arg = %d\n",arg);
	//设置cd4501三个管脚的电平
	gpio_direction_output(ads8320->data->cd4051a, arg>>0&0x01);
	gpio_direction_output(ads8320->data->cd4051b, arg>>1&0x01);
	gpio_direction_output(ads8320->data->cd4051c, arg>>2&0x01);

	return 0;
}

static int ads8320_open(struct inode *inode, struct file *filp)
{
	struct ads8320_data *ads8320;
	int status = -ENXIO;
	mutex_lock(&device_list_lock);

	list_for_each_entry(ads8320, &device_list, device_entry) {
		if (ads8320->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if(!ads8320->buffer) {
			ads8320->buffer = kmalloc(3, GFP_KERNEL);// (5+16)bit需要3个字节
			if(!ads8320->buffer) {
				printk(&ads8320->spi->dev, "open/ENOMEM\n");	
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			ads8320->users++;
			filp->private_data = ads8320;
			nonseekable_open(inode, filp);
		}
	}else
		printk("ads8320, nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);

	return status;
}

static int ads8320_release(struct inode *inode, struct file *filp)
{
	struct ads8320_data	*ads8320;
	int			status = 0;

	mutex_lock(&device_list_lock);
	ads8320 = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	ads8320->users--;
	if (!ads8320->users) {
		int dofree;
		
		spin_lock_irq(&ads8320->spi_lock);
		dofree = (ads8320->spi == NULL);
		spin_unlock_irq(&ads8320->spi_lock);

		if(dofree)
			kfree(ads8320);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct class *ads8320_class;

static const struct file_operations ads8320_fops = {
	.owner	= THIS_MODULE,
	.read	= ads8320_read,
	.unlocked_ioctl = ads8320_ioctl,
	.open	= ads8320_open,
	.release= ads8320_release,
};

static void ads8320_setup_cdev(struct ads8320_data *ads8320,int index)
{
	int err, devno = MKDEV(ads8320_major, index);

	/*初始化cdev成员,并建立cdev和file_operations之间的连接*/
	cdev_init(&ads8320->cdev, &ads8320_fops);
	ads8320->cdev.owner = THIS_MODULE;

	/*向系统添加一个cdev设备*/
	err = cdev_add(&ads8320->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %s adding ads8320%d", err, index);
}


static int __devinit ads8320_probe(struct spi_device *spi)
{
	struct ads8320_data	*ads8320;
	int			status;

	/* Allocate driver data */
	ads8320 = kzalloc(sizeof(*ads8320), GFP_KERNEL);
	if (!ads8320)
		return -ENOMEM;

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	ads8320->devt = MKDEV(ads8320_major, minor);//次设备号是0
	printk("major:%d, minor:%d\n",ads8320_major,  minor);
	//申请主设备号
	if(ads8320_major) 
		status = register_chrdev_region(ads8320->devt, 1, "ads8320");
	else {
		status  = alloc_chrdev_region(&ads8320->devt, 0, 1, "ads8320");	
		ads8320_major = MAJOR(ads8320->devt);
	}
	if(status < 0) 
		return status;

	ads8320_setup_cdev(ads8320, 0);

	//取出平台数据,cd4051的管脚配置
	ads8320->data = (struct cd4051_platform_data *)spi->dev.platform_data;
	if(ads8320->data) {
		printk("cd4051a => %d\n", ads8320->data->cd4051a);
		printk("cd4051b => %d\n", ads8320->data->cd4051b);
		printk("cd4051c => %d\n", ads8320->data->cd4051c);
	}

	/* Initialize the driver data */
	ads8320->spi = spi;
	spin_lock_init(&ads8320->spi_lock);//初始化自旋锁
	mutex_init(&ads8320->buf_lock);	//初始化互斥体
	INIT_LIST_HEAD(&ads8320->device_entry);//实始化链表头


	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	struct device *dev;
	/*创建一个设备节点，名为ads83201.0*/
	dev = device_create(ads8320_class, &spi->dev, ads8320->devt,
			ads8320, "ads8320%d.%d",
			spi->master->bus_num, spi->chip_select);
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&ads8320->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, ads8320);
	else
		kfree(ads8320);

	return status;

}

static int __devexit ads8320_remove(struct spi_device *spi)
{

	struct ads8320_data	*ads8320 = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&ads8320->spi_lock);
	ads8320->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&ads8320->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&ads8320->device_entry);
	clear_bit(MINOR(ads8320->devt), minors);
	cdev_del(&ads8320->cdev);//在系统中删除cdev
	device_destroy(ads8320_class, ads8320->devt);
	unregister_chrdev_region(ads8320->devt, 1);//释放原先申请的设备号
	if (ads8320->users == 0)
		kfree(ads8320);
	
	mutex_unlock(&device_list_lock);

	return 0;
}


static struct spi_driver ads8320_driver = {
	.driver = {
		.name	= "ads8320",
		.owner	= THIS_MODULE,
	},
	.probe		= ads8320_probe,
	.remove		= __devexit_p(ads8320_remove),

};

static int __init ads8320_init(void)
{
	int status;
	/*注册一个类，/sys/class/ads8320, 使mdev可以在/dev/目录下创建设备节点*/
	ads8320_class = class_create(THIS_MODULE, "ads8320");
	if (IS_ERR(ads8320_class)) {
		return PTR_ERR(ads8320_class);
	}

	status = spi_register_driver(&ads8320_driver);
	if (status < 0) {
		class_destroy(ads8320_class);
	}
	else
		printk(KERN_NOTICE "ads8320 driver register success.\n" );
	return status;
}

static void __exit ads8320_exit(void)
{
	spi_unregister_driver(&ads8320_driver);
	class_destroy(ads8320_class);
	printk(KERN_NOTICE "ads8320 driver unregister success.\n" );
}
module_init(ads8320_init);
module_exit(ads8320_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
