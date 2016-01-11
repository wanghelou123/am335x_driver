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
#include <linux/delay.h>
#include <linux/klha_ads124x.h>
#include "ads1247.h"

#define pr_fmt(fmt) "ads124x: " fmt
#define ADS124X_MAJOR			154	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
static unsigned char reg_buf[15];
static char* reg_name[15]={"MUX0", "VBIAS", "MUX1", "SYS0", "OFC0", "OFC1", "OFC2", "FSC0", "FSC1", "FSC2", "IDAC0", "IDAC1", "GPIOCFG", "GPIODIR", "GPIODAT"};


static	int ads124x_major = ADS124X_MAJOR;
unsigned long		minor = 0;

static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct ads124x_data {
	struct cdev			cdev;
	dev_t				devt;
	spinlock_t			spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct mutex		buf_lock;
	unsigned			users;
	u8					*buffer;
	unsigned int		ad_value;
	struct ads124x_platform_data *data;
};

struct omap2_mcspi_cs {
	void __iomem        *base;
	unsigned long       phys;
	int         word_len;
	struct list_head    node;
	/* Context save and restore shadow register */
	u32         chconf0;
};


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
//声明
void ads124x_complete(void *arg);
ssize_t ads124x_sync(struct ads124x_data *ads124x, struct spi_message *message);

void ads124x_write_char(struct ads124x_data *ads124x, unsigned char  tx_byte)
{
	struct spi_message m;
	ads124x->buffer[0] = tx_byte;
	struct spi_transfer t = {
		.tx_buf = ads124x->buffer,
		.len = 1,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ads124x_sync(ads124x, &m);
	udelay(2);
}

unsigned char ads124x_read_char(struct ads124x_data *ads124x)
{
	unsigned char rx_byte=0x00;
	int ret = 0;
	struct spi_message m;

	//ads124x->buffer[0]= 0;
	struct spi_transfer t = {
		//.tx_buf = ads124x->buffer,
		.rx_buf = &rx_byte,
		.len = 1,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = ads124x_sync(ads124x, &m);
	pr_info("ret = %d\n", ret);
	//rx_byte = ads124x->buffer[0];

	return rx_byte;
}

void  ads124x_read_reg(struct ads124x_data *ads124x, unsigned char reg,unsigned char num)
{
	unsigned char i;
	//下降沿写数据
	ads124x->spi->mode= SPI_MODE_1;
	spi_setup(ads124x->spi);

#if 1
	struct spi_message m;
	ads124x->buffer[0] = ADS1247_COMMAND_RREG_1ST|reg;
	ads124x->buffer[1] = ADS1247_COMMAND_RREG_2ND|num;
	struct spi_transfer t = {
		.tx_buf = ads124x->buffer,
		.len = 2,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ads124x_sync(ads124x, &m);
#endif

	//上升沿读数据
	ads124x->spi->mode= SPI_MODE_1;
	spi_setup(ads124x->spi);
#if 1
	memset(ads124x->buffer, 0, 20);
	struct spi_transfer t2 = {
		.rx_buf = ads124x->buffer,
		.len = 15,
		.delay_usecs = 2,
	};

	struct spi_message m2;
	spi_message_init(&m2);
	spi_message_add_tail(&t2, &m2);

	ads124x_sync(ads124x, &m2);
#endif

#if 0
	for(i=0;i<15;i++)
		reg_buf[i] = ads124x_read_char();
	udelay(2);
#endif

	for(i=0;i<15;i++)
		//pr_info("reg: %s\t = %02x\n", reg_name[i], reg_buf[i]);
		pr_info("reg: %s\t = %02x\n", reg_name[i], ads124x->buffer[i]);
}

void ads124x_write_reg(struct ads124x_data *ads124x, unsigned char reg,unsigned char data)
{
	//SCLK低电平空闲，下降沿写到DIN
	ads124x->spi->mode= SPI_MODE_1;
	//ads124x->spi->max_speed_hz = 4096000;
	//ads124x->spi->max_speed_hz = 2048000;
	spi_setup(ads124x->spi);
	//ads124x_write_char(ADS1247_COMMAND_WREG_1ST|reg);
	//ads124x_write_char(ADS1247_COMMAND_WREG_2ND|0X00);
	//ads124x_write_char(data);
	struct spi_message m;
	ads124x->buffer[0] = ADS1247_COMMAND_WREG_1ST|reg;
	ads124x->buffer[1] = ADS1247_COMMAND_WREG_2ND|0X00;
	ads124x->buffer[2] = data;
	struct spi_transfer t = {
		.tx_buf = ads124x->buffer,
		.len = 3,
		.delay_usecs = 2,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ads124x_sync(ads124x, &m);
	//udelay(2);
}

void ads124x_write_cmd(struct ads124x_data *ads124x, unsigned char cmd)
{
	ads124x_write_char(ads124x, cmd);
	mdelay(401);
	udelay(2);
}

static void ads124x_reset(unsigned gpio)
{
	gpio_set_value(gpio, 0);
	udelay(10);
	gpio_set_value(gpio, 1);
	mdelay(1);
}

void ads124x_read_once(struct ads124x_data *ads124x)
{
	gpio_set_value(ads124x->data->START, 1);
	udelay(2);
	gpio_set_value(ads124x->data->START, 0);
}

void ads124x_data_ready(struct ads124x_data *ads124x)
{
	int i=0;
	while(gpio_get_value(ads124x->data->DRDY) == 1){
		i++;
		ndelay(1);
	}
	//pr_info("i = %d\n", i);

	//上升沿读数据
	ads124x->spi->mode= SPI_MODE_1;
	spi_setup(ads124x->spi);
	memset(ads124x->buffer, 0, 20);
	struct spi_transfer t = {
		.rx_buf = ads124x->buffer,
		.len = 3,
		.delay_usecs = 2,
	};

	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ads124x_sync(ads124x, &m);

	ads124x->ad_value=0;
	ads124x->ad_value |= ads124x->buffer[0]<<16;
	ads124x->ad_value |= ads124x->buffer[1]<<8;
	ads124x->ad_value |= ads124x->buffer[2]<<0;
	if(ads124x->ad_value & 0x800000) {
		ads124x->ad_value |= 0xFF000000;	
	}

	//pr_info("ad_value = 0x%08x", ads124x->ad_value);
}


void ads124x_complete(void *arg)
{
	complete(arg);//唤醒完成量
}

ssize_t ads124x_sync(struct ads124x_data *ads124x, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);//声明并且初始化一个"完成量"
	int status;

	message->complete = ads124x_complete; //传输完成后的回调函数
	message->context = &done;//回调函数的参数

	spin_lock_irq(&ads124x->spi_lock);
	if (ads124x->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(ads124x->spi, message);
	spin_unlock_irq(&ads124x->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);//等待一个完成量被唤醒
		status = message->status;
		if (status == 0)
			status = message->actual_length; //传输完成的位数
	}

	return status;
}
static inline ssize_t ads124x_sync_read(struct ads124x_data *ads124x, size_t len)
{
	struct spi_transfer t = {
		.rx_buf = ads124x->buffer,
		.len	= 3, /* 片选后4.5 ~ 5 个周期用于采样, 16 bit为数据，总共需要3个字节, 一个时钟周期是416ns */
	};

	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return ads124x_sync(ads124x, &m);
}

static ssize_t ads124x_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int err = 0;
	count = 4;
	struct ads124x_data	*ads124x;
	ads124x = filp->private_data;
	ads124x_read_once(ads124x);
	ads124x_data_ready(ads124x);

	err = copy_to_user(buf, (void*)&ads124x->ad_value, 4);

	if(err)
		return -EFAULT;
	else
		return min(4, count);

}

static long ads124x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ads124x_data	*ads124x;
	ads124x = filp->private_data;

	cmd = _IOC_NR(cmd)  -1;
	if(cmd <0 || cmd > 7)
		return -EINVAL;

	ads124x_reset(ads124x->data->RESET);
	if(cmd<=5)
		/*ads1248, PGA=1, SPS=640*/
		ads124x_write_reg(ads124x, ADS1247_ADDR_SYS0, ADS1248_PGA_USE|ADS1248_SPS_USE);
	else
		/*ads1247, PGA=64, SPS=320*/
		ads124x_write_reg(ads124x, ADS1247_ADDR_SYS0, ADS1247_PGA_USE|ADS1247_SPS_USE);

	/*内部基准源打开，使用内部基准源，正常测量模式*/
	ads124x_write_reg(ads124x, ADS1247_ADDR_MUX1, 0x30);

	switch(cmd){
		case 0:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL1);
			break;
		case 1:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL2);
			break;
		case 2:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL3);
			break;
		case 3:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL4);
			break;
		case 4:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL5);
			break;
		case 5:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL6);
			break;
		case 6:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL7);
			break;
		case 7:
			ads124x_write_reg(ads124x, ADS1247_ADDR_MUX0, CHANNEL8);
			break;
	}
	mdelay(25);

	return 0;
}

static int ads124x_open(struct inode *inode, struct file *filp)
{
	struct ads124x_data *ads124x;
	int status = -ENXIO;
	mutex_lock(&device_list_lock);

	list_for_each_entry(ads124x, &device_list, device_entry) {
		if (ads124x->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	//pr_info("%s, status=%d\n", __func__, status);

	if (status == 0) {
		ads124x->users++;
		filp->private_data = ads124x;
		nonseekable_open(inode, filp);
	}else
		printk("ads124x, nothing for minor %d\n", iminor(inode));
#if 0
	//ads124x_write_cmd(ads124x, ADS1247_COMMAND_RESET);
	ads124x_reset(ads124x->data->RESET);
	ads124x_write_cmd(ads124x, ADS1247_COMMAND_SYSGCAL);
	if(MINOR(ads124x->devt) == 0){
		/*ads1248*/
		ads124x_write_reg(ads124x, ADS1247_ADDR_SYS0, ADS1248_PGA_USE|ADS1248_SPS_USE);
		pr_info("operate ads1248.\n");
	}
	else{
		/*ads1247*/
		ads124x_write_reg(ads124x, ADS1247_ADDR_SYS0, ADS1247_PGA_USE|ADS1247_SPS_USE);
		pr_info("operate ads1247.\n");
	}
	ads124x_write_reg(ads124x, ADS1247_ADDR_MUX1, 0x30);
	ads124x_read_reg(ads124x, ADS1247_ADDR_MUX0, 15);
#endif

	mutex_unlock(&device_list_lock);

	return status;
}

static int ads124x_release(struct inode *inode, struct file *filp)
{
	struct ads124x_data	*ads124x;
	int			status = 0;

	mutex_lock(&device_list_lock);
	ads124x = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	ads124x->users--;
	if (!ads124x->users) {
		int dofree;

		spin_lock_irq(&ads124x->spi_lock);
		dofree = (ads124x->spi == NULL);
		spin_unlock_irq(&ads124x->spi_lock);

		if(dofree)
			kfree(ads124x);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct class *ads124x_class;

static const struct file_operations ads124x_fops = {
	.owner	= THIS_MODULE,
	.read	= ads124x_read,
	.unlocked_ioctl = ads124x_ioctl,
	.open	= ads124x_open,
	.release= ads124x_release,
};

static void ads124x_setup_cdev(struct ads124x_data *ads124x,int index)
{
	int err, devno = MKDEV(ads124x_major, index);

	/*初始化cdev成员,并建立cdev和file_operations之间的连接*/
	cdev_init(&ads124x->cdev, &ads124x_fops);
	ads124x->cdev.owner = THIS_MODULE;

	/*向系统添加一个cdev设备*/
	err = cdev_add(&ads124x->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding ads124x%d", err, index);
}


static int __devinit ads124x_probe(struct spi_device *spi)
{
	int			status;
	struct device *dev;
	struct ads124x_data	*ads124x;

	/* Allocate driver data */
	ads124x = kzalloc(sizeof(*ads124x), GFP_KERNEL);
	if (!ads124x)
		return -ENOMEM;

	ads124x->buffer = kzalloc(20, GFP_KERNEL);// (5+16)bit需要3个字节
	if(!ads124x->buffer) {
		printk(&ads124x->spi->dev, "open/ENOMEM\n");	
		status = -ENOMEM;
	}

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	ads124x->devt = MKDEV(ads124x_major, minor);//次设备号是0
	printk("major:%d, minor:%d\n",ads124x_major,  minor);
	//申请主设备号
	if(ads124x_major) 
		status = register_chrdev_region(ads124x->devt, 1, "ads124x");
	else {
		status  = alloc_chrdev_region(&ads124x->devt, 0, 1, "ads124x");	
		ads124x_major = MAJOR(ads124x->devt);
	}
	if(status < 0) 
		return status;

	ads124x_setup_cdev(ads124x, minor);

	//取出平台数据,管脚配置
	ads124x->data = (struct ads124x_platform_data *)spi->dev.platform_data;
	if(ads124x->data) {
		printk("RESET => %d\n", ads124x->data->RESET);
		printk("START => %d\n", ads124x->data->START);
		printk("DRDY => %d\n", ads124x->data->DRDY);
	}
	gpio_request(ads124x->data->RESET, NULL);
	gpio_request(ads124x->data->START, NULL);
	gpio_request(ads124x->data->DRDY, NULL);
	gpio_direction_output(ads124x->data->RESET, 1);
	gpio_direction_output(ads124x->data->START, 1);
	gpio_direction_input(ads124x->data->DRDY);

	/* Initialize the driver data */
	ads124x->spi = spi;
	spin_lock_init(&ads124x->spi_lock);//初始化自旋锁
	mutex_init(&ads124x->buf_lock);	//初始化互斥体
	INIT_LIST_HEAD(&ads124x->device_entry);//实始化链表头


	/*debug*/
	pr_info("spi->mode = %d\n", spi->mode);
	pr_info("spi->max_speed_hz = %d\n", spi->max_speed_hz);
	pr_info("spi->controller_state->base = %8x\n", ((struct omap2_mcspi_cs*)spi->controller_state)->base);


	/*初始化ads124x*/
	mdelay(30);
	ads124x_reset(ads124x->data->RESET);
#if 0
	/*PGA=64, SPS=320*/
	ads124x_write_reg(ads124x, ADS1247_ADDR_SYS0, ADS1248_PGA_USE|ADS1248_SPS_USE);
	/*内部基准源打开，使用内部基准源，正常测量模式*/
	ads124x_write_reg(ads124x, ADS1247_ADDR_MUX1, 0x30);
	/*拉低START*/
	gpio_direction_output(ads124x->data->START, 0);

	/*系统增益校准*/
	ads124x_write_cmd(ads124x, ADS1247_COMMAND_SYSGCAL);

	gpio_direction_output(ads124x->data->START, 0); 
	mdelay(20); 

#endif

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	/*创建一个设备节点，名为ads124x1.0*/
	dev = device_create(ads124x_class, &spi->dev, ads124x->devt,
			ads124x, "ads124x%d.%d",
			spi->master->bus_num, spi->chip_select);
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&ads124x->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, ads124x);
	else
	{
		kfree(ads124x->buffer);
		kfree(ads124x);
	}

	return status;

}

static int __devexit ads124x_remove(struct spi_device *spi)
{

	pr_info("%s\n",__func__ );
	struct ads124x_data	*ads124x = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&ads124x->spi_lock);
	ads124x->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&ads124x->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&ads124x->device_entry);
	clear_bit(MINOR(ads124x->devt), minors);
	cdev_del(&ads124x->cdev);//在系统中删除cdev
	device_destroy(ads124x_class, ads124x->devt);
	unregister_chrdev_region(ads124x->devt, 1);//释放原先申请的设备号
	if (ads124x->users == 0){
		gpio_free(ads124x->data->RESET);
		gpio_free(ads124x->data->START);
		gpio_free(ads124x->data->DRDY);
		kfree(ads124x->buffer);
		kfree(ads124x);
	}

	mutex_unlock(&device_list_lock);

	return 0;
}


static struct spi_driver ads124x_driver = {
	.driver = {
		.name	= "ads124x",
		.owner	= THIS_MODULE,
	},
	.probe		= ads124x_probe,
	.remove		= __devexit_p(ads124x_remove),

};

static int __init ads124x_init(void)
{
	int status;
	/*注册一个类，/sys/class/ads124x, 使mdev可以在/dev/目录下创建设备节点*/
	ads124x_class = class_create(THIS_MODULE, "ads124x");
	if (IS_ERR(ads124x_class)) {
		return PTR_ERR(ads124x_class);
	}

	status = spi_register_driver(&ads124x_driver);
	if (status < 0) {
		class_destroy(ads124x_class);
	}
	else
		printk(KERN_NOTICE "ads124x driver register success.\n" );
	return status;
}

static void __exit ads124x_exit(void)
{
	spi_unregister_driver(&ads124x_driver);
	class_destroy(ads124x_class);
	printk(KERN_NOTICE "ads124x driver unregister success.\n" );
}


module_init(ads124x_init);
module_exit(ads124x_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("whl");
MODULE_DESCRIPTION("ADS1247/48 SPI driver for am335x.");
