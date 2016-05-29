/********************************************************
【平    台】龙丘CORTEX-M4开发板/系统板
【编    写】龙丘
【Designed】by Chiu Sir
【E-mail  】chiusir@yahoo.cn
【软件版本】V1.0
【最后更新】2013年3月16日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【dev.env.】IAR6.30
【Target  】CORTEX-M4
【Crystal 】50.000Mhz
【busclock】50.000MHz
【pllclock】 ?MHz
------------------------------------------------   
------------------------------------------------
接线如下：
将D6引脚设置为模式3，即UART0_RX
将D7引脚设置为模式3，即UART0_TX

  SA0接地
  SCL B2引脚启用I2C0 SCL功能  本程序是模拟实现
  SDA B3引脚启用I2C0 SDA功能  本程序是模拟实现
  3.3V
  GND

使用PORTA BIT14,15,16,17流水灯程序

2012-04-20 测试通过
直接输出加速度数值

三轴陀螺仪模块上直接接地了
三轴加速度模块上可以跳线选择
*********************************************************/	
#include <stdio.h>
#include "MMA8451_I2C.h"
#include "Cpu.h"
#include "Events.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
#include "Bit1.h"
#include "BitIoLdd1.h"
#include "standcontrol.h"
#include "FTM0_PWM_CTL_Motor.h"
#include "DMAT1.h"
#include "FTM1_capture_pfre.h"
#include "DMA1.h"
#include "PCLR.h"
#include "BitIoLdd2.h"
#include "count_time.h"
#include "BitIoLdd3.h"
#include "camera_loop.h"
#include "left_Wheel_DIR.h"
#include "BitIoLdd4.h"
#include "Right_Wheel_DIR.h"
#include "BitIoLdd5.h"
#include "I2CSCL.h"
#include "BitIoLdd6.h"
#include "I2CSDA.h"
#include "BitIoLdd7.h"
#include "delay.h"
#include "boma1.h"
#include "BitsIoLdd2.h"
#include "SM1.h"
#include "boma2.h"
#include "BitsIoLdd3.h"
#include "SDHC.h"
#include "BitIoLdd10.h"
#include "BitIoLdd11.h"
#include "BitIoLdd12.h"
#include "CS.h"
#include "BitIoLdd13.h"
#include "MMA8451_SCL.h"
#include "MMA8451_SDA.h"
#include "DC.h"
#include "BitIoLdd8.h"
#include "BitIoLdd9.h"
#include "Reset.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"
#include "LPTMR0_capture_fre.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "math.h"

//端口位定义，B2 SCL;B3 SDA
#define SDA     GPIOC_PDIR&(0x0001<<11) //IO口输入 SDA
#define SDA0()  MMA8451_SDA_ClrVal()	//IO口输出低电平
#define SDA1()  MMA8451_SDA_SetVal()	//IO口输出高电平  
#define SCL0()  MMA8451_SCL_ClrVal()	//IO口输出低电平
#define SCL1()  MMA8451_SCL_SetVal()	//IO口输出高电平
#define DIR_OUT()    MMA8451_SDA_SetOutput()       //输出方向
#define DIR_IN()     MMA8451_SDA_SetInput()  //输入方向

//SA0必须接地

void iicdelay()
{
  /* int i;   
	for(i=0;i<500;i++) 	   
	      asm("nop");*/
}

//内部数据定义
unsigned char IIC_ad_main; //器件从地址	    
unsigned char IIC_ad_sub;  //器件子地址	   
unsigned char *IIC_buf;    //发送|接收数据缓冲区	    
unsigned char IIC_num;     //发送|接收数据个数	     

#define ack 1      //主应答
#define no_ack 0   //从应答	 

//nop指令个数定义   
//#define iicdelay() {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}  
void IIC_start(void){
	SCL0();
	SDA1();
	iicdelay();
	SCL1();
	iicdelay();
	SDA0();
	iicdelay();
	SCL0();
}
//************************************************
//送停止位 SDA=0->1
void IIC_stop(void){
	SCL0();
	iicdelay();
	SDA0();
	iicdelay();
	SCL1();
	iicdelay();
	SDA1();
	iicdelay();
	SCL0();
}
//************************************************
//主应答(包含ack:SDA=0和no_ack:SDA=0)
void IIC_ack_main(unsigned char ack_main){
	SCL0();
	if(ack_main) SDA0(); //ack主应答
	else SDA1(); //no_ack无需应答
	iicdelay();
	SCL1();
	iicdelay();
	SCL0();
}
//*************************************************
//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
void send_ch(unsigned char c){
	unsigned char i;
	for(i=0;i<8;i++){
		SCL0();
		if((c<<i) & 0x80)SDA1(); //判断发送位
		else SDA0();
		iicdelay();
		SCL1();
		iicdelay();
		SCL0();
	}
	iicdelay();
	SDA1(); //发送完8bit，释放总线准备接收应答位
	iicdelay();
	SCL1();
	iicdelay(); //sda上数据即是从应答位              
	SCL0(); //不考虑从应答位|但要控制好时序
}
//**************************************************
//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//return: uchar型1字节
unsigned char read_ch(void)
{
   unsigned char i;
   unsigned char c;
	c=0;
	SCL0();
	iicdelay();
	SDA1(); //置数据线为输入方式
	DIR_IN();
	for(i=0;i<8;i++){
		iicdelay();
		SCL0(); //置时钟线为低，准备接收数据位
		iicdelay();
		SCL1(); //置时钟线为高，使数据线上数据有效
		iicdelay();
		c<<=1;
		if(SDA) c+=1; //读数据位，将接收的数据存c
	}
	SCL0();
	DIR_OUT();
	return c;
}
//***************************************************
//向无子地址器件发送单字节数据
void send_to_ch(unsigned char ad_main,unsigned char c){
	IIC_start();
	send_ch(ad_main); //发送器件地址
	send_ch(c); //发送数据c
	IIC_stop();
}
//***************************************************
//向有子地址器件发送多字节数据
void send_to_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main); //发送器件地址
	send_ch(ad_sub); //发送器件子地址
	for(i=0;i<num;i++){
		send_ch(*buf); //发送数据*buf
		buf++;
	}
	IIC_stop();
}
//***************************************************
//从无子地址器件读单字节数据
//function:器件地址，所读数据存在接收缓冲区当前字节
void read_from_ch(unsigned char ad_main,unsigned char *buf){
	IIC_start();
	send_ch(ad_main); //发送器件地址
	*buf=read_ch();
	IIC_ack_main(no_ack); //无需应答<no_ack=0>
	IIC_stop();
}
//***************************************************
//从有子地址器件读多个字节数据
//function:
void read_from_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main);
	send_ch(ad_sub);
	for(i=0;i<num-1;i++){
		*buf=read_ch();
		IIC_ack_main(ack); //主应答<ack=1>
		buf++;
	}
	*buf=read_ch();
	buf++; //本次指针调整无意义，目的是操作后buf指向下一地址
	IIC_ack_main(no_ack); //无需应答<no_ack=0>
	IIC_stop();
}




unsigned char MMA845x_readch(unsigned char address)
{
unsigned char ret = 100;
	IIC_start();		//启动
	send_ch(MMA845x_IIC_ADDRESS);	//写入设备ID及写信号
	send_ch(address);	//X地址
	IIC_start();		//重新发送开始
	send_ch(MMA845x_IIC_ADDRESS+1);	//写入设备ID及读信
	ret = read_ch();	//读取一字节
	IIC_stop();

	return ret;
}

//写入
void MMA845x_writecha(unsigned char address, unsigned char thedata)
{
	IIC_start();		//启动
	send_ch(MMA845x_IIC_ADDRESS);	//写入设备ID及写信号
	send_ch(address);	//X地址
	send_ch(thedata);	//写入设备ID及读信
	IIC_stop();
}

//初始化
//初始化为指定模式
void Init_MMA8451()
{	
	MMA845x_writecha(CTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);	
	iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); 
	MMA845x_writecha(XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G
	iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay();
	MMA845x_writecha(CTRL_REG1, (ACTIVE_MASK+ASLP_RATE_20MS+DATA_RATE_5MS)&(~FREAD_MASK)); //激活状态   14bit
	iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay();
}


short read_Y_value()
{
	byte z_value[2];
	word value;
//	short my_return_value;
//	read_reg_MMA8451(0x05,0x02u,&z_value[0]);
	z_value[0]=MMA845x_readch(0X05);
	z_value[1]=MMA845x_readch(0X06);
	value=(word)z_value[0];
	value=value<<6;
	value|=(word)(z_value[1]>>2);
		 
		 if(value&0x2000)
			{
			 value|=0xc000;
			}	 
		else
			value&=0x7fff;
		// my_return_value=(short)value;
		// return my_return_value;
		return (short)value;
	
}


void get_acc(float *z_angle)
{
	double  input_rate;
	short  z_value;
	z_value=read_Y_value();
	float  z_float_value;
	float  b_srcB=57.29577951;
	input_rate=(double)z_value/4095;
	if(input_rate>1)
		input_rate=1;
	if(input_rate<-1)
		input_rate=-1;
	//*z_angle=(float)asin(input_rate);
	z_float_value=(float)asin(input_rate);
	*z_angle=(-1)*z_float_value*b_srcB;
}

