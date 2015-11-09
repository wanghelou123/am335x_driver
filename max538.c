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
#include <linux/klha_max538.h>

#define pr_fmt(fmt) "max538: " fmt
#define DEVICE_NAME "max538"//继电器
unsigned SPI_MOSI;
unsigned SPI_CLK;

static int major = 0;
static struct class * max538_class;
struct max538_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct max538_platform_data	*data;
};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_max538_lock);
static const struct of_device_id of_max538_match[] = {
	{.compatible = DEVICE_NAME},
	{},
};


//向DA寄存器发送16位数据其中低12有效
int send_2_bytes(unsigned spi_cs, int data)
{
	//pr_info("clk:%d, mosi:%d, cs:%d, data:0x%x\n",SPI_CLK, SPI_MOSI,  spi_cs, data);
	unsigned char bit_counter=16;
	/*发送16位数据计数变量*/
	unsigned short temp=0x00;
	gpio_set_value(SPI_CLK,0);
	ndelay(20);	/*tCSH0:SCLK拉低后需要保持的时间。*/
	gpio_set_value(spi_cs,0);
	/*片选MAX538有效*/
	ndelay(30);	/*tCSS: CS下降后需要保持的时间。*/
	do
	{
		temp=data;		
		if((temp&0x8000)==0x8000)
		{
			gpio_set_value(SPI_MOSI,1);
			ndelay(50); /*tDS:DIN setup time.*/
			/*printk("1 ");*/
		}
		else
		{			gpio_set_value(SPI_MOSI,0);
			ndelay(50); /*tDS:DIN setup time.*/
			/*printf("0 ");*/
		}
		gpio_set_value(SPI_CLK,1);
		/*DIN设置完1位，SCLK上升，发送数据*/
		ndelay(50);		/*tCH:SCLK为1保持*/
		gpio_set_value(SPI_CLK,0);
		ndelay(50);		/*tCL: SCLK为0保持*/
		temp=data<<1;	/*下一位数据*/
		data=temp;
		bit_counter--;
	}while(bit_counter);
	gpio_set_value(spi_cs,1);
	/*所有数据都发送完成后，CS无效，此时模拟电压输出*/

	ndelay(30);
	/*tCSW:CS保持1的最短时间。*/

	return 2;
}

static ssize_t max538_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned short data=(unsigned short)0;
	struct max538_data		*pdata;
	pdata = filp->private_data;
	data = (buf[1]<<8) + buf[0];

	if(data > 0x1000)
		return -EINVAL;

	send_2_bytes(pdata->data->cs_array[0], data);

	return 2;
}

static int max538_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct max538_data		*pdata;
	pdata = filp->private_data;


	if(cmd > 0x1000)
		return -EINVAL;

	send_2_bytes(pdata->data->cs_array[arg], cmd);

	return 0;
}

static int max538_open(struct inode *inode, struct file *filp)
{
	struct max538_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_max538_lock);
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

	mutex_unlock(&device_max538_lock);

	return status;
}

static int max538_release(struct inode *inode, struct file *filp)
{
	struct max538_data		*pdata;
	pdata = filp->private_data;

	mutex_lock(&device_max538_lock);
	pdata->users--;
	mutex_unlock(&device_max538_lock);

	return 0;
}

static struct file_operations dev_fops = {
	.owner =     THIS_MODULE,
	.unlocked_ioctl =     max538_ioctl,
	.write			=	max538_write,
	.open			=	max538_open,
	.release		=	max538_release,
}; 

static void max538_setup_cdev(struct max538_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &dev_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding %s\n", err, DEVICE_NAME);
}

static int __devinit max538_probe(struct platform_device *pdev)
{
	int i=0;
	int status;
	struct max538_data * pdata;
	struct device *dev;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	max538_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}
	SPI_CLK	= pdata->data->clk;
	SPI_MOSI = pdata->data->mosi;
	pr_info("SPI_CLK:gpio => %d\n", SPI_CLK);
	pr_info("SPI_MOSI:gpio => %d\n", SPI_MOSI);
	gpio_request(SPI_CLK,NULL);
	gpio_request(SPI_MOSI,NULL);
	gpio_direction_output(SPI_CLK, 0);
	gpio_direction_output(SPI_MOSI, 0);

	//打印管脚信息
	for(i = 0; i<pdata->data->nums; i++) {
		gpio_request(pdata->data->cs_array[i], NULL);
		pr_info("CS%d gpio => %d\n", i, pdata->data->cs_array[i]);
		gpio_direction_output(pdata->data->cs_array[i], 1);
		send_2_bytes(pdata->data->cs_array[i],0);/*发送数据*/
	}

	/*创建一个设备节点，名为/dev/max538 */
	dev = device_create(max538_class, &pdev->dev, pdata->devt, pdata, DEVICE_NAME);
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;

	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}

	return status;
}

static int __devexit max538_remove(struct platform_device *pdev)
{
	struct max538_data *pdata;
	pdata = platform_get_drvdata(pdev);
	cdev_del(&pdata->cdev);
	device_destroy(max538_class, pdata->devt);
	unregister_chrdev_region(pdata->devt, 1);
	pr_info("pdata->users=>%d\n", pdata->users);
	if(pdata->users == 0) {
		int i;
		for(i=0; i<pdata->data->nums ; i++) {
			gpio_free(pdata->data->cs_array[i]);
		}
		kfree(pdata);
	}

	return 0;
}

static struct platform_driver max538_driver = {
	.probe = max538_probe,
	.remove = __devexit_p(max538_remove),
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_max538_match,
	},
};

static int __init dev_init(void)
{
	int status;

	max538_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(max538_class)) {
		return PTR_ERR(max538_class);
	}
	status = platform_driver_register(&max538_driver);

	if(status < 0 ) {
		pr_err( "register %s to platform failed!\n" );
		class_destroy(max538_class);
	}else
		pr_info(" driver register success.\n");

	return status;
}

static void __exit dev_exit(void)
{
	platform_driver_unregister(&max538_driver);
	class_destroy(max538_class);
	printk(KERN_NOTICE "%s driver unregister success.\n", DEVICE_NAME);
}
module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("WHL");
