/************************************************
  DS18B20的驱动，platform总线形式
  维护记录：  2015-10-28 V1.1   
  设备名称：	ds18b20
 *************************************************/
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
#include <linux/klha_ds18b20.h>

#define HIGH 1
#define LOW 0

//定义DS18B20ROM指令
#define    DS18B20_ReadROM        0x33    //读ROM
#define    ds18b20_matchROM       0x55    //匹配ROM
#define    DS18B20_SkipROM        0xCC    //跳过ROM
#define    DS18B20_SearchROM      0xF0    //搜索ROM
#define    DS18B20_AlarmROM       0xEC    //报警搜索

//定义DS18B20存储器操作命令
#define    DS18B20_WriteSCR       0x4E    //写暂存存储器
#define    DS18B20_ReadSCR        0xBE    //读暂存存储器
#define    DS18B20_CopySCR        0x48    //复制暂存存储器

//定义DS18B20功能操作指令
#define    DS18B20_ConvertTemp    0x44    //温度变换
#define    DS18B20_RecallEP       0xB8    //重新调出
#define    DS18B20_ReadPower      0xB4    //读电源

#define DEVICE_NAME	"ds18b20"    /* 设备名称 */		

static int major = 0;
static struct class * ds18b20_class;
struct ds18b20_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct ds18b20_platform_data *data;

};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_ds18b20_lock);
static const struct of_device_id of_ds18b20_match[] = {
	{.compatible = DEVICE_NAME},
	{},
};

unsigned short temp;
unsigned char DS18B20_ID[8] = {0};
unsigned DQ_PIN;

unsigned char ds18b20_reset(void)
{	
	gpio_direction_output(DQ_PIN, LOW);//将DQ拉低，发送复位脉冲
	udelay(700);
	gpio_direction_output(DQ_PIN, HIGH);//拉高总线
	udelay(60);
	gpio_direction_input(DQ_PIN);
	udelay(10);

	if(gpio_get_value(DQ_PIN)!=0)	//测试复位是否成功
		printk("ds18b20 reset failed!\n");

	return 0;
}

unsigned char ds18b20_readbyte(void)                                                             
{                                                                                                
	unsigned char i,dat=0;                                                                       

	for(i=0;i<8;i++)
	{                                                                                            
		gpio_direction_output(DQ_PIN, LOW);                                                      
		udelay(2);
		gpio_direction_output(DQ_PIN, HIGH); //add                                               
		gpio_direction_input(DQ_PIN);
		if(gpio_get_value(DQ_PIN))
			dat |= 1<<i;    

		udelay(60);                                                                              
	}
	return dat;
}

void ds18b20_writebyte(unsigned char dat) 
{
	unsigned char j;
	unsigned char testb;
	for(j=0;j<8;j++)
	{
		udelay(1);//每次写时序间隔 1us
		gpio_direction_output(DQ_PIN, LOW);
		udelay(15);
		if((dat>>j)&0x01)   //写1
		{
			gpio_direction_output(DQ_PIN, HIGH);
		}
		else		//写0
		{
			//因为开始已经拉低总线，所以这里不用再次操作
		}
		udelay(60);
		gpio_direction_output(DQ_PIN, HIGH);
	}
}

void ds18b20_readid(void)
{
	int i=0;
	//udelay(1);
	ds18b20_reset();
	udelay(100);
	ds18b20_writebyte(DS18B20_ReadROM);
	udelay(4);
	DS18B20_ID[0] = ds18b20_readbyte();
	DS18B20_ID[1] = ds18b20_readbyte();
	DS18B20_ID[2] = ds18b20_readbyte();
	DS18B20_ID[3] = ds18b20_readbyte();
	DS18B20_ID[4] = ds18b20_readbyte();
	DS18B20_ID[5] = ds18b20_readbyte();
	DS18B20_ID[6] = ds18b20_readbyte();
	DS18B20_ID[7] = ds18b20_readbyte();

	printk("ds18b20 ID: ");
	for(i=0; i<8; i++) {
		printk("%.2X", DS18B20_ID[7-i]);
	}printk("\n");
}

void ds18b20_read_power(void)
{
	int power_mode=0;
	ds18b20_reset();
	udelay(100);
	ds18b20_writebyte(DS18B20_ReadPower);
	udelay(4);
	power_mode = ds18b20_readbyte(); 
	if(power_mode) {
		printk("ds18b20 power mode is externally power supply.\n");	
	}else {
		printk("ds18b20 power mode is autoeciousness power supply.\n");
	}
}

void ds18b20_match(void)
{
	ds18b20_writebyte(ds18b20_matchROM);
	ds18b20_writebyte(DS18B20_ID[0]);
	ds18b20_writebyte(DS18B20_ID[1]);
	ds18b20_writebyte(DS18B20_ID[2]);
	ds18b20_writebyte(DS18B20_ID[3]);
	ds18b20_writebyte(DS18B20_ID[4]);
	ds18b20_writebyte(DS18B20_ID[5]);
	ds18b20_writebyte(DS18B20_ID[6]);
	ds18b20_writebyte(DS18B20_ID[7]);
}

void DS18B20_TmpChange(void)
{
	while(ds18b20_reset());
	udelay(100);/*delay(1);*/
	ds18b20_writebyte(DS18B20_SkipROM);  // address all drivers on bus
	ds18b20_writebyte(DS18B20_ConvertTemp);  //  initiates a single temperature conversion
}

unsigned short DS18B20_Temperature(void)
{
	unsigned char a,b;
	while(ds18b20_reset());
	udelay(100);
	ds18b20_writebyte(DS18B20_SkipROM);
	ds18b20_writebyte(DS18B20_ReadSCR);
	udelay(4);
	a=ds18b20_readbyte();
	b=ds18b20_readbyte();
	temp=b;
	temp<<=8;             //two byte  compose a int variable
	temp=temp|a;

	return temp;
}

static int ds18b20_open(struct inode *inode, struct file *filp)
{
	struct ds18b20_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_ds18b20_lock);
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

	mutex_unlock(&device_ds18b20_lock);

	printk("SMDK2416-DS18B20 Driver Open is Called!\n");
	DS18B20_TmpChange();
	//mdelay(1000);
	DS18B20_Temperature();

	return status;
}

static int ds18b20_release(struct inode *inode, struct file *filp)
{
	struct ds18b20_data		*pdata;
	printk("SMDK2416-DS18B20 Driver Release Called!\n");
	pdata = filp->private_data;

	mutex_lock(&device_ds18b20_lock);
	pdata->users--;
	mutex_unlock(&device_ds18b20_lock);

	return 0;
}

static int ds18b20_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	unsigned short ds18b20_val;
	int ret;

	DS18B20_TmpChange();
	msleep(500);
	ds18b20_val =  DS18B20_Temperature();
	//printk("ds18b20_val=0x%x\n", ds18b20_val);
	if(ds18b20_val == 0xffff)
		ds18b20_val = 0;
	ret = copy_to_user(buf, (void *)&ds18b20_val, sizeof(ds18b20_val));

	return ret;
}


//控制DS18B20电源引脚的断开或连接
static long ds18b20_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ds18b20_data		*pdata;
	pdata = filp->private_data;
	switch(cmd){
		case 0:
			gpio_direction_output(pdata->data->power, HIGH);
			break;

		case 1:
			gpio_direction_output(pdata->data->power, LOW);
			break;
	}

	return 0;
}

static struct file_operations ds18b20_fops =
{
	.owner  =   THIS_MODULE,
	.open   =   ds18b20_open, 
	.release =  ds18b20_release,
	.unlocked_ioctl  =   ds18b20_ioctl,
	.read   =   ds18b20_read,
};

static void ds18b20_setup_cdev(struct ds18b20_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &ds18b20_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding %s\n", err, DEVICE_NAME);
}

static int __devinit ds18b20_probe(struct platform_device *pdev)
{
	int status;
	struct ds18b20_data * pdata;
	struct device *dev;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	ds18b20_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}

	//打印管脚信息
	gpio_request(pdata->data->data, "data");
	gpio_request(pdata->data->power, "power");
	DQ_PIN =  pdata->data->data;
	printk(KERN_NOTICE "%s data(DQ_PIN) pin  => %d\n", DEVICE_NAME,  pdata->data->data);
	printk(KERN_NOTICE "%s power pin => %d\n", DEVICE_NAME,  pdata->data->power);

	/*创建一个设备节点，名为/dev/ds18b20 */
	dev = device_create(ds18b20_class, &pdev->dev, pdata->devt, pdata, DEVICE_NAME);
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;

	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}

	gpio_direction_output(pdata->data->power, LOW);//power on
	gpio_direction_output(pdata->data->data, LOW);//power on

	return status;
}

static int __devexit ds18b20_remove(struct platform_device *pdev)
{
	struct ds18b20_data *pdata;
	pdata = platform_get_drvdata(pdev);
	cdev_del(&pdata->cdev);
	device_destroy(ds18b20_class, pdata->devt);
	unregister_chrdev_region(pdata->devt, 1);
	printk("pdata->users=>%d\n", pdata->users);
	if(pdata->users == 0) {
		gpio_free(pdata->data->data);
		gpio_free(pdata->data->power);
		kfree(pdata);
	}

	return 0;
}

static struct platform_driver ds18b20_driver = {
	.probe = ds18b20_probe,
	.remove = __devexit_p(ds18b20_remove),
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_ds18b20_match,
	},
};

static int __init dev_init(void)
{
	int status;

	ds18b20_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(ds18b20_class)) {
		return PTR_ERR(ds18b20_class);
	}
	status = platform_driver_register(&ds18b20_driver);

	ds18b20_reset();
	ds18b20_readid();
	ds18b20_read_power();

	if(status < 0 ) {
		printk(KERN_ERR "register %s to platform failed!\n", DEVICE_NAME);
		class_destroy(ds18b20_class);
	}else
		printk(KERN_NOTICE "%s driver register success.\n", DEVICE_NAME);

	return status;
}

static void __exit dev_exit(void)
{
	platform_driver_unregister(&ds18b20_driver);
	class_destroy(ds18b20_class);
	printk(KERN_NOTICE "%s driver unregister success.\n", DEVICE_NAME);
}
module_init(dev_init);
module_exit(dev_exit);

MODULE_AUTHOR("whl");		
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DS18B20 Driver");	

