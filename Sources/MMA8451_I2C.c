/********************************************************
��ƽ    ̨������CORTEX-M4������/ϵͳ��
����    д������
��Designed��by Chiu Sir
��E-mail  ��chiusir@yahoo.cn
������汾��V1.0
�������¡�2013��3��16��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
��dev.env.��IAR6.30
��Target  ��CORTEX-M4
��Crystal ��50.000Mhz
��busclock��50.000MHz
��pllclock�� ?MHz
------------------------------------------------   
------------------------------------------------
�������£�
��D6��������Ϊģʽ3����UART0_RX
��D7��������Ϊģʽ3����UART0_TX

  SA0�ӵ�
  SCL B2��������I2C0 SCL����  ��������ģ��ʵ��
  SDA B3��������I2C0 SDA����  ��������ģ��ʵ��
  3.3V
  GND

ʹ��PORTA BIT14,15,16,17��ˮ�Ƴ���

2012-04-20 ����ͨ��
ֱ��������ٶ���ֵ

����������ģ����ֱ�ӽӵ���
������ٶ�ģ���Ͽ�������ѡ��
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

//�˿�λ���壬B2 SCL;B3 SDA
#define SDA     GPIOC_PDIR&(0x0001<<11) //IO������ SDA
#define SDA0()  MMA8451_SDA_ClrVal()	//IO������͵�ƽ
#define SDA1()  MMA8451_SDA_SetVal()	//IO������ߵ�ƽ  
#define SCL0()  MMA8451_SCL_ClrVal()	//IO������͵�ƽ
#define SCL1()  MMA8451_SCL_SetVal()	//IO������ߵ�ƽ
#define DIR_OUT()    MMA8451_SDA_SetOutput()       //�������
#define DIR_IN()     MMA8451_SDA_SetInput()  //���뷽��

//SA0����ӵ�

void iicdelay()
{
  /* int i;   
	for(i=0;i<500;i++) 	   
	      asm("nop");*/
}

//�ڲ����ݶ���
unsigned char IIC_ad_main; //�����ӵ�ַ	    
unsigned char IIC_ad_sub;  //�����ӵ�ַ	   
unsigned char *IIC_buf;    //����|�������ݻ�����	    
unsigned char IIC_num;     //����|�������ݸ���	     

#define ack 1      //��Ӧ��
#define no_ack 0   //��Ӧ��	 

//nopָ���������   
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
//��ֹͣλ SDA=0->1
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
//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
void IIC_ack_main(unsigned char ack_main){
	SCL0();
	if(ack_main) SDA0(); //ack��Ӧ��
	else SDA1(); //no_ack����Ӧ��
	iicdelay();
	SCL1();
	iicdelay();
	SCL0();
}
//*************************************************
//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
void send_ch(unsigned char c){
	unsigned char i;
	for(i=0;i<8;i++){
		SCL0();
		if((c<<i) & 0x80)SDA1(); //�жϷ���λ
		else SDA0();
		iicdelay();
		SCL1();
		iicdelay();
		SCL0();
	}
	iicdelay();
	SDA1(); //������8bit���ͷ�����׼������Ӧ��λ
	iicdelay();
	SCL1();
	iicdelay(); //sda�����ݼ��Ǵ�Ӧ��λ              
	SCL0(); //�����Ǵ�Ӧ��λ|��Ҫ���ƺ�ʱ��
}
//**************************************************
//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|IIC_ack_main()ʹ��
//return: uchar��1�ֽ�
unsigned char read_ch(void)
{
   unsigned char i;
   unsigned char c;
	c=0;
	SCL0();
	iicdelay();
	SDA1(); //��������Ϊ���뷽ʽ
	DIR_IN();
	for(i=0;i<8;i++){
		iicdelay();
		SCL0(); //��ʱ����Ϊ�ͣ�׼����������λ
		iicdelay();
		SCL1(); //��ʱ����Ϊ�ߣ�ʹ��������������Ч
		iicdelay();
		c<<=1;
		if(SDA) c+=1; //������λ�������յ����ݴ�c
	}
	SCL0();
	DIR_OUT();
	return c;
}
//***************************************************
//�����ӵ�ַ�������͵��ֽ�����
void send_to_ch(unsigned char ad_main,unsigned char c){
	IIC_start();
	send_ch(ad_main); //����������ַ
	send_ch(c); //��������c
	IIC_stop();
}
//***************************************************
//�����ӵ�ַ�������Ͷ��ֽ�����
void send_to_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main); //����������ַ
	send_ch(ad_sub); //���������ӵ�ַ
	for(i=0;i<num;i++){
		send_ch(*buf); //��������*buf
		buf++;
	}
	IIC_stop();
}
//***************************************************
//�����ӵ�ַ���������ֽ�����
//function:������ַ���������ݴ��ڽ��ջ�������ǰ�ֽ�
void read_from_ch(unsigned char ad_main,unsigned char *buf){
	IIC_start();
	send_ch(ad_main); //����������ַ
	*buf=read_ch();
	IIC_ack_main(no_ack); //����Ӧ��<no_ack=0>
	IIC_stop();
}
//***************************************************
//�����ӵ�ַ����������ֽ�����
//function:
void read_from_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main);
	send_ch(ad_sub);
	for(i=0;i<num-1;i++){
		*buf=read_ch();
		IIC_ack_main(ack); //��Ӧ��<ack=1>
		buf++;
	}
	*buf=read_ch();
	buf++; //����ָ����������壬Ŀ���ǲ�����bufָ����һ��ַ
	IIC_ack_main(no_ack); //����Ӧ��<no_ack=0>
	IIC_stop();
}




unsigned char MMA845x_readch(unsigned char address)
{
unsigned char ret = 100;
	IIC_start();		//����
	send_ch(MMA845x_IIC_ADDRESS);	//д���豸ID��д�ź�
	send_ch(address);	//X��ַ
	IIC_start();		//���·��Ϳ�ʼ
	send_ch(MMA845x_IIC_ADDRESS+1);	//д���豸ID������
	ret = read_ch();	//��ȡһ�ֽ�
	IIC_stop();

	return ret;
}

//д��
void MMA845x_writecha(unsigned char address, unsigned char thedata)
{
	IIC_start();		//����
	send_ch(MMA845x_IIC_ADDRESS);	//д���豸ID��д�ź�
	send_ch(address);	//X��ַ
	send_ch(thedata);	//д���豸ID������
	IIC_stop();
}

//��ʼ��
//��ʼ��Ϊָ��ģʽ
void Init_MMA8451()
{	
	MMA845x_writecha(CTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);	
	iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); 
	MMA845x_writecha(XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G
	iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay();
	MMA845x_writecha(CTRL_REG1, (ACTIVE_MASK+ASLP_RATE_20MS+DATA_RATE_5MS)&(~FREAD_MASK)); //����״̬   14bit
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

