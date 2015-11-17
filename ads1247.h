//ADS1247内部命令
#define ADS1247_COMMAND_WAKEUP      0X00    //唤醒
#define ADS1247_COMMAND_SLEEP       0X02    //睡眠
#define ADS1247_COMMAND_RESET       0X06    //复位
#define ADS1247_COMMAND_NOP         0XFF    //空指令
#define ADS1247_COMMAND_RDATA       0X12    //读一次数据
#define ADS1247_COMMAND_RDATAC      0X14    //连续读数据
#define ADS1247_COMMAND_SDATAC      0X16    //停止连续读数据
#define ADS1247_COMMAND_RREG_1ST    0X20    //读寄存器字节1
#define ADS1247_COMMAND_RREG_2ND    0X00    //读寄存器字节2
#define ADS1247_COMMAND_WREG_1ST    0X40    //写寄存器字节1
#define ADS1247_COMMAND_WREG_2ND    0X00    //写寄存器字节2
#define ADS1247_COMMAND_SYSOCAL     0X60    //系统偏移校准
#define ADS1247_COMMAND_SYSGCAL     0X61    //系统增益校准
#define ADS1247_COMMAND_SELFOCAL    0X62    //自偏移校准

//ADS1247内部寄存器地址
#define ADS1247_ADDR_MUX0           0X00
#define ADS1247_ADDR_VBIAS          0X01
#define ADS1247_ADDR_MUX1           0X02
#define ADS1247_ADDR_SYS0           0X03
#define ADS1247_ADDR_OFC0           0X04
#define ADS1247_ADDR_OFC1           0X05
#define ADS1247_ADDR_OFC2           0X06
#define ADS1247_ADDR_FSC0           0X07
#define ADS1247_ADDR_FSC1           0X08
#define ADS1247_ADDR_FSC2           0X09
#define ADS1247_ADDR_IDAC0          0X0A
#define ADS1247_ADDR_IDAC1          0X0B

/*采集通道号*/
#define	CHANNEL1					0x07	//4~20mA
#define	CHANNEL2					0x0F	//4~20mA
#define	CHANNEL3					0x17	//0~5V
#define	CHANNEL4					0x1F	//0~5V
#define	CHANNEL5					0x27	//0~5V
#define	CHANNEL6					0x2F	//0~5V
#define	CHANNEL7					0x01	//采集第一路差分信号0~20mV
#define	CHANNEL8					0x13	//采集第二路差分信号0~20mV

//ADS1247支持的增益列表  
#define ADS1247_GAIN_1          0x00  
#define ADS1247_GAIN_2          0x10  
#define ADS1247_GAIN_4          0x20  
#define ADS1247_GAIN_8          0x30  
#define ADS1247_GAIN_16         0x40  
#define ADS1247_GAIN_32         0x50
#define ADS1247_GAIN_64         0x60  
#define ADS1247_GAIN_128        0x70  
#define ADS1247_PGA_USE			ADS1247_GAIN_32
#define ADS1248_PGA_USE			ADS1247_GAIN_1

//ADS1247支持的转换速率列表  
#define ADS1247_SPS_5           0x00  
#define ADS1247_SPS_10          0x01  
#define ADS1247_SPS_20          0x02  
#define ADS1247_SPS_40          0x03  
#define ADS1247_SPS_80          0x04  
#define ADS1247_SPS_160         0x05  
#define ADS1247_SPS_320         0x06  
#define ADS1247_SPS_640         0x07  
#define ADS1247_SPS_1000        0x08  
#define ADS1247_SPS_2000        0x09  
#define	ADS1248_SPS_USE			ADS1247_SPS_640
#define	ADS1247_SPS_USE			ADS1247_SPS_320

//ADS1247转换模式设置  
#define ADS1247_MODE_SINGLECOV      0x00        //单次转换模式  
#define ADS1247_MODE_CONTINUOUS     0x01        //连续转换模式  

#define	CSSC	10	//CS low to first SCLK high (set up time)
#define	SCCS	7	//SCLK low to CS high (hold time)
#define RHSC	600	//RESET high to SPI communication start, 0.6ms
#define DTS		244 //DRDY falling edge to SCLK rising edge

struct ads124x_data;
void ads124x_data_ready(struct ads124x_data *ads124x);
unsigned char ads124x_read_char(struct ads124x_data *ads124x);
void ads124x_write_reg(struct ads124x_data *ads124x,unsigned char reg,unsigned char data);
void ads124x_write_cmd(struct ads124x_data *ads124x, unsigned char cmd);
void ads124x_read_reg(struct ads124x_data *ads124x, unsigned char reg,unsigned char num);
void ads124x_read_once(struct ads124x_data *ads124x);
void ads124x_write_char(struct ads124x_data *ads124x,unsigned char  TX_byte);
static void ads124x_reset(unsigned gpio);
static int __init ads124x_init(void);
static void __exit ads124x_exit(void);
static int ads124x_open(struct inode *inode,struct file *file);
static int ads124x_release(struct inode *inode, struct file *file);
static ssize_t ads124x_read(struct file *filp, char __user *buff, size_t count, loff_t *offp);
static long ads124x_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
