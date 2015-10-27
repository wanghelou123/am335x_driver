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
#include <linux/klha_lcd1602.h>
#define DEVICE_NAME "LCD1602"
unsigned LCDRS;
unsigned LCDRW;
unsigned LCDE;
unsigned DB0;	 
unsigned DB1; 
unsigned DB2; 
unsigned DB3; 
unsigned DB4;	 
unsigned DB5;	 
unsigned DB6;	 
unsigned DB7;	 

static int major = 0;
static struct class * lcd1602_class;
struct lcd1602_data {
	struct cdev			cdev;
	dev_t				devt;
	struct list_head	device_entry;
	unsigned			users;
	struct lcd1602_platform_data *data;
};
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_lcd1602_lock);
static const struct of_device_id of_lcd1602_match[] = {
	{.compatible = DEVICE_NAME},
	{},
};

static int address_count = 0;
// 循环检测LCD忙标志(BF)，直到其值为0，方可执行下一指令   
void lcd_wait_until_finish(void)  
{  
	lcd1602_rs_out(CMD);		// RS置0，读状态时RS需置低电平      
	lcd1602_rw_out(READ);		// RW置1，状态为读           
	lcd1602_en_out(HIGH);		// E置1，读取信息  
	udelay(20);  

	int count = 0;
	while(lcd1602_bf_in()){
		if(count++ > 10)
			break;
	}

	// 循环直至BF=0  
	lcd1602_en_out(LOW);		// E重置为0  
	udelay(20);  
} 


// 向LCD写命令字  
void lcd_command(unsigned char cmd)  
{  
	//printk("command: 0x%x\n", cmd);
	lcd_wait_until_finish(); // 等待执行完毕  
	lcd1602_rs_out(CMD);     // RS置0，写入命令字  
	lcd1602_rw_out(WRITE);     // RW置0，状态为写  

	udelay(20);  
	lcd1602_en_out(HIGH);
	lcd1602_bus_write(cmd);  // 将命令字cmd送入LCD的数据端口
	lcd1602_en_out(LOW);
	udelay(20);  
} 


// 设置显示位置(即写入显示地址)，x,y均从0开始  
void lcd_goto_xy(unsigned char x, unsigned char y)  
{  
	unsigned char p;       // p为字符显示位置,即DDRAM中的地址  
	if (y==0)  
	{  
		p = 0x00 + x;      // (0,0)显示位置为0x00  
	}  
	else  
	{  
		p = 0x40 + x;      // (0,1)显示位置为0x40  
	}  
	lcd_command(p + 0x80); // 写入显示地址时DB7须为高电平,加0x80  
} 

// 写字符(传入的参数实际为所需显示字符的地址,即液晶字符产生器中字符的地址)    
void lcd_putc(unsigned char c)  
{  
	lcd_wait_until_finish(); // 等待执行完毕  
	lcd1602_rs_out(DATA);    // RS置1，写入数据  
	lcd1602_rw_out(WRITE);     // RW置0，状态为写  
	udelay(20);  
	lcd1602_en_out(HIGH);
	lcd1602_bus_write(c);	 // 将命令字cmd送入LCD的数据端口    
	lcd1602_en_out(LOW);
	udelay(20);  
} 
unsigned char lcd_get_char()
{
	lcd_wait_until_finish(); // 等待执行完毕  
	lcd1602_rs_out(DATA);		// RS置0，读状态时RS需置低电平      
	lcd1602_rw_out(READ);		// RW置1，状态为读           
	lcd1602_en_out(HIGH);		// E置1，读取信息  
	udelay(20);  
	unsigned char data=lcd1602_bus_read();
	// 循环直至BF=0  
	lcd1602_en_out(LOW);		// E重置为0  
	udelay(20);  
	//printk("read data => 0x%2x\n", data);

	return data;
}

// 指定位置写字符  
void lcd_xy_putc(unsigned char x, unsigned char y, unsigned char c)  
{  
	lcd_goto_xy(x,y);
	udelay(100); //要延迟下
	lcd_putc(c);  
}  

// 写字符串   
void lcd_puts(unsigned char *s)  
{  
	while(*s)  
	{  
		lcd_putc(*s);
		//udelay(100); //要延迟下
		s++;  
	}  
}  

// 指定位置写字符串     
void lcd_xy_puts(unsigned char x, unsigned char y, unsigned char *s)  
{  
	lcd_goto_xy(x, y);
	udelay(100); //要延迟下
	lcd_puts(s);  
} 

void lcd1602_clear(void)
{
	lcd_command(0x01);
	mdelay(10);
}

/*
 * hl 为0：输出低电平,非0：输出高电平
 */
void lcd1602_en_out(int hl) //使能
{
	gpio_direction_output(LCDE, (hl == 0) ? 0 : 1);
}


//读写1602控制,H:读1602,L:写入1602
void lcd1602_rw_out(int hl) 
{
	gpio_direction_output(LCDRW, (hl == 0) ? 0 : 1);
}


//输入指令数据选择
void lcd1602_rs_out(int hl) 
{
	gpio_direction_output(LCDRS, (hl == 0) ? 0 : 1);
}


//向1602的8根数据总线写入数据
void lcd1602_bus_write(unsigned char c)
{
	//printk("write data: %c ==> 0x%x\n", c, c);
	gpio_direction_output(DB0, (c>>0)&1);
	gpio_direction_output(DB1, (c>>1)&1);
	gpio_direction_output(DB2, (c>>2)&1);
	gpio_direction_output(DB3, (c>>3)&1);
	gpio_direction_output(DB4, (c>>4)&1);
	gpio_direction_output(DB5, (c>>5)&1);
	gpio_direction_output(DB6, (c>>6)&1);
	gpio_direction_output(DB7, (c>>7)&1);
}

//从1602的8根数据总线读入数据
unsigned char lcd1602_bus_read(void)
{
	unsigned char data = 0;

	//读取8位数据
	data |= gpio_get_value(DB0)<<0;
	data |= gpio_get_value(DB1)<<1;
	data |= gpio_get_value(DB2)<<2;
	data |= gpio_get_value(DB3)<<3;
	data |= gpio_get_value(DB4)<<4;
	data |= gpio_get_value(DB5)<<5;
	data |= gpio_get_value(DB6)<<6;
	data |= gpio_get_value(DB7)<<7;


	return data;
}
//BF=1表示液晶显示器忙,返回值就是bf的状态
int lcd1602_bf_in(void)
{
	return (lcd1602_bus_read() & 0x80);
}

int lcd1602_open(struct inode *inode, struct file *filp)
{
	struct lcd1602_data *pdata;
	int status = -ENXIO;

	mutex_lock(&device_lcd1602_lock);
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

	mutex_unlock(&device_lcd1602_lock);

	return status;
}

int lcd1602_release(struct inode *inode, struct file *filp)
{
	struct lcd1602_data		*pdata;
	pdata = filp->private_data;

	mutex_lock(&device_lcd1602_lock);
	pdata->users--;
	mutex_unlock(&device_lcd1602_lock);

	return 0; 
}

static ssize_t lcd1602_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char buffer[33]={0};
	int i=0;
	for(i=0; i<16;i++) {
		lcd_goto_xy(i,0);
		buffer[i] = lcd_get_char();
		printk("addr:%x %c ==> %x\n",0x80|i,  buffer[i], buffer[i]);
	}

	for(i=0; i<16;i++) {
		lcd_goto_xy(i,0x40);
		buffer[16+i] = lcd_get_char();
		printk("addr:%x %c ==> %x\n",0x80|(0x40+i),  buffer[16+i], buffer[16+i]);
	}


	copy_to_user(buf, buffer,32);
#if 0
	for(i=0; i<0x27;i++){
		lcd_goto_xy(i,0);
		printk("0x%2x %c ==> %x\n", i, lcd_get_char(), lcd_get_char() );
	}
	for(i=0; i<0x27;i++){
		lcd_goto_xy(i,0x40);
		printk("0x%2x %c ==> %x\n", i+0x40, lcd_get_char(),  lcd_get_char());
	}
#endif

	return 32;
}

static ssize_t lcd1602_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char line1[17]={0};
	unsigned char line2[17]={0};

	if(size > 32)size = 32;

	lcd1602_clear();

	if(size<16) {
		memcpy(line1, buf, size);
	} else {
		memcpy(line1, buf, 16);
		memcpy(line2, buf+16, size-16);
	}

	lcd_puts(line1);
	lcd_goto_xy(0, 0x40);
	lcd_puts(line2);

	return size;
}

#define PUT_CHAR 0
#define CLEAR 1
static long my_LCD1602_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	cmd =  _IOC_NR(cmd); 

	switch (cmd){
		case PUT_CHAR:
			printk("put char.\n");
			lcd_putc((unsigned char)arg);
			break;
		case CLEAR:
			printk("clear the screen.\n");
			lcd1602_clear();
			break;
		default:
			return -EINVAL;
	}
}

static struct file_operations lcd1602_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = my_LCD1602_ioctl,
	.read = lcd1602_read,
	.write = lcd1602_write,
	.open = lcd1602_open,
	.release = lcd1602_release,
};

static void lcd1602_setup_cdev(struct lcd1602_data *pdata, int index)
{
	int err, devno = MKDEV(major, index);
	cdev_init(&pdata->cdev, &lcd1602_fops);

	err = cdev_add(&pdata->cdev, devno, 1);
	if(err)
		printk(KERN_NOTICE "Error %d adding %s\n", err, DEVICE_NAME);
}

static int __devinit lcd1602_probe(struct platform_device *pdev)
{
	int status;
	struct lcd1602_data * pdata;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata)
		return -ENOMEM;

	status = alloc_chrdev_region(&pdata->devt, 0, 1, DEVICE_NAME);
	if(status < 0)
		return status;
	major = MAJOR(pdata->devt);

	lcd1602_setup_cdev(pdata, 0);

	pdata->data = pdev->dev.platform_data;
	if(!pdata->data) {
		printk(KERN_ERR "NOT FIND GPIO.");
		return -ENODEV;
	}

	//打印管脚信息
	gpio_request(pdata->data->lcdrs, "lcdrs");
	gpio_request(pdata->data->lcdrw, "lcdrw");
	gpio_request(pdata->data->lcde, "lcde");
	gpio_request(pdata->data->db0, "db0");
	gpio_request(pdata->data->db1, "db1");
	gpio_request(pdata->data->db2, "db2");
	gpio_request(pdata->data->db3, "db3");
	gpio_request(pdata->data->db4, "db4");
	gpio_request(pdata->data->db5, "db5");
	gpio_request(pdata->data->db6, "db6");
	gpio_request(pdata->data->db7, "db7");
	LCDRS = pdata->data->lcdrs;
	LCDRW = pdata->data->lcdrw;
	LCDE = pdata->data->lcde;
	DB0	= pdata->data->db0;
	DB1	= pdata->data->db1;
	DB2	= pdata->data->db2;
	DB3	= pdata->data->db3;
	DB4	= pdata->data->db4;
	DB5	= pdata->data->db5;
	DB6	= pdata->data->db6;
	DB7	= pdata->data->db7;
	printk(KERN_NOTICE "%s  lcdrs=> %d\n", DEVICE_NAME,  pdata->data->lcdrs);
	printk(KERN_NOTICE "%s  lcdrw=> %d\n", DEVICE_NAME,  pdata->data->lcdrw);
	printk(KERN_NOTICE "%s  lcde=> %d\n", DEVICE_NAME,  pdata->data->lcde);
	printk(KERN_NOTICE "%s  db0=> %d\n", DEVICE_NAME,  pdata->data->db0);
	printk(KERN_NOTICE "%s  db1=> %d\n", DEVICE_NAME,  pdata->data->db1);
	printk(KERN_NOTICE "%s  db2=> %d\n", DEVICE_NAME,  pdata->data->db2);
	printk(KERN_NOTICE "%s  db3=> %d\n", DEVICE_NAME,  pdata->data->db3);
	printk(KERN_NOTICE "%s  db4=> %d\n", DEVICE_NAME,  pdata->data->db4);
	printk(KERN_NOTICE "%s  db5=> %d\n", DEVICE_NAME,  pdata->data->db5);
	printk(KERN_NOTICE "%s  db6=> %d\n", DEVICE_NAME,  pdata->data->db6);
	printk(KERN_NOTICE "%s  db7=> %d\n", DEVICE_NAME,  pdata->data->db7);

	/*创建一个设备节点，名为/dev/lcd1602 */
	struct device *dev;
	dev = device_create(lcd1602_class, &pdev->dev, pdata->devt, pdata, DEVICE_NAME);
	status = IS_ERR(dev) ?PTR_ERR(dev) : 0;
	
	if (status == 0) {
		list_add(&pdata->device_entry, &device_list);
		platform_set_drvdata(pdev, pdata);	
	} else {
		kfree(pdata);
	}
	
	return status;
}

static int __devexit lcd1602_remove(struct platform_device *pdev)
{
	struct lcd1602_data *pdata;
	pdata = platform_get_drvdata(pdev);
	cdev_del(&pdata->cdev);
	device_destroy(lcd1602_class, pdata->devt);
	unregister_chrdev_region(pdata->devt, 1);
	printk("pdata->users=>%d\n", pdata->users);
	if(pdata->users == 0) {
		gpio_free(pdata->data->lcdrs);
		gpio_free(pdata->data->lcdrw);
		gpio_free(pdata->data->lcde);
		gpio_free(pdata->data->db0);
		gpio_free(pdata->data->db1);
		gpio_free(pdata->data->db2);
		gpio_free(pdata->data->db3);
		gpio_free(pdata->data->db4);
		gpio_free(pdata->data->db5);
		gpio_free(pdata->data->db6);
		gpio_free(pdata->data->db7);

		kfree(pdata);
	}
}

static struct platform_driver lcd1602_driver = {
	.probe = lcd1602_probe,
	.remove = __devexit_p(lcd1602_remove),
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_lcd1602_match,
	},
};

static int __init lcd1602_init(void)
{
	int status;

	lcd1602_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(lcd1602_class)) {
			return PTR_ERR(lcd1602_class);
	}
	status = platform_driver_register(&lcd1602_driver);

	//初始化LCD
	lcd_command(0X38);//8位总线，双行显示, 5*7点阵列字符
	printk("%s: 8 bit bus, double line show, 5*7 character\n", DEVICE_NAME);
	lcd_command(0X0C);//显示打开
	printk("%s: show ON\n", DEVICE_NAME);
	lcd_command(0X06);//设置光标移动的方向为右
	printk("%s: set cursor right!\n", DEVICE_NAME);
	lcd_command(0X01);//清屏
	printk("%s: clear the screen!\n", DEVICE_NAME);

	lcd_goto_xy(0,0);			 // 字符位置：(4,0) , 5*7点阵列字符
	lcd_puts(" Welcome to use");    // 显示字符
	lcd_goto_xy(0,40);
	lcd_puts("collector");

	if(status < 0 ) {
		printk(KERN_ERR "register %s to platform failed!\n", DEVICE_NAME);
		class_destroy(lcd1602_class);
	}else
		printk(KERN_NOTICE "%s driver register success.\n", DEVICE_NAME);

	return status;
}

static void __exit lcd1602_exit(void)
{
	platform_driver_unregister(&lcd1602_driver);
	class_destroy(lcd1602_class);
	printk(KERN_NOTICE "%s driver unregister success.\n", DEVICE_NAME);
}

module_init(lcd1602_init);
module_exit(lcd1602_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wang He Lou");
MODULE_DESCRIPTION("s3c2416 LCD1602 driver");
