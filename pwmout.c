/************************************************
  PWM 输出的驱动，在am3352发板上做测试
  维护记录：  2014-3-6  V1.0   
  维护记录：  2015-11-17   V1.1
  linux内核：3.2.0
  设备名称: pwmout
 *************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/clk.h>

#define DEVICE_NAME "pwmout"    	/* 设备名称 */     

static int pwmout_open(struct inode *inode,struct file *file)
{

	printk("pwmout Driver open Called!\n");

	return 0;
}

static int pwmout_release(struct inode *inode, struct file *file)
{
	printk("pwmout Driver release Called!\n");

	return 0;
}

static struct semaphore lock;                            //定义信号量 lock

static void PWM0_Set_Freq( unsigned long duty_cycle)     
{
	struct file *fp;  
	char buf[10];
	memset(buf, 0, 10);
	mm_segment_t fs;  
	loff_t pos;  

    duty_cycle=100-duty_cycle;
	sprintf(buf, "%d", duty_cycle);
	pr_info("%s: duty_cycle=%s\n", __func__, buf);  
	fp = filp_open("/sys/class/pwm/ecap.1/request", O_RDWR | O_CREAT, 0644);  
	if (IS_ERR(fp)) {  
		printk("create file error\n");  
		return ;  
	}  
	fs = get_fs();  
	set_fs(KERNEL_DS);  
	pos = 0;  
	vfs_write(fp, "1", sizeof("1"), &pos);  
	filp_close(fp, NULL);  


	fp = filp_open("/sys/class/pwm/ecap.1/period_freq", O_RDWR | O_CREAT, 0644);  
	if (IS_ERR(fp)) {  
		printk("create file error\n");  
		return ;  
	}  
	vfs_write(fp, "5", sizeof("5"), &pos);  
	filp_close(fp, NULL);  


	fp = filp_open("/sys/class/pwm/ecap.1/duty_percent", O_RDWR | O_CREAT, 0644);  
	if (IS_ERR(fp)) {  
		printk("create file error\n");  
		return ;  
	}  
	vfs_write(fp, buf, sizeof(buf), &pos);  
	filp_close(fp, NULL);  


	fp = filp_open("/sys/class/pwm/ecap.1/run", O_RDWR | O_CREAT, 0644);  
	if (IS_ERR(fp)) {  
		printk("create file error\n");  
		return ;  
	}  
	vfs_write(fp, "1", sizeof("0"), &pos);  
	filp_close(fp, NULL);  

	set_fs(fs);  
}

/*
 *cmd是占空比，0~100; 脉宽为200ms, 5HZ
 */
static long pwmout_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	down(&lock);

	cmd = _IOC_NR(cmd);
	//返回错误信息：请求的资源不可用
	if(cmd <0 || cmd >100)
		return -EINVAL;

	PWM0_Set_Freq(cmd);

	up(&lock);

	return 0;
}

static struct file_operations pwmout_fops = {
	.owner			=	THIS_MODULE,
	.unlocked_ioctl =   pwmout_ioctl,
	.open			=	pwmout_open,
	.release	 	=	pwmout_release,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	/*misc设备的设备号 值为255*/
	.name = DEVICE_NAME,
	/*这个设备的名称 值为pwmout*/
	.fops = &pwmout_fops,
	/* pwmout_fops 是上节所述的文件系统接口*/
};/*定义一个简单的小型设备。*/

static int __init dev_init(void){
	int ret;
	/*将misc注册到linux系统中*/
	ret = misc_register(&misc);
	sema_init(&lock,1);
	printk(DEVICE_NAME " initialized\n");

	return ret;
}

static void __exit dev_exit(void)
{

	struct file *fp;  
	mm_segment_t fs;  
	loff_t pos;  
	pr_info("%s\n", __func__);  
	fp = filp_open("/sys/class/pwm/ecap.1/run", O_RDWR | O_CREAT, 0644);  
	if (IS_ERR(fp)) {  
		printk("create file error\n");  
		return -1;  
	}  
	fs = get_fs();  
	set_fs(KERNEL_DS);  
	pos = 0;  
	vfs_write(fp, "0", sizeof("0"), &pos);  
	filp_close(fp, NULL);  
	set_fs(fs);  

	misc_deregister(&misc);
	printk(DEVICE_NAME " removed\n");
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wang He Lou");
