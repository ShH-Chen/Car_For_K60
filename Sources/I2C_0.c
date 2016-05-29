
/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "I2C_0.h"
#include "global.h"
#include "arm_math.h"

float gyro_x_bias=0.62;
float gyro_y_bias=-0.9;
float gyro_z_bias=0.5;

LDD_TDeviceData* I2C0DeviceDataPtr;
void delay(void)
{    int i;
	 for(i=0;i<=3000;i++);
}


/*************************************************************************
陀螺仪初始化及寄存器设置
*************************************************************************/

void Init_I2C0(void)
{
	I2C0DeviceDataPtr=I2C0_Init(NULL);
}

LDD_TError Init_L3D(void)
{	LDD_TError err;
	//取消powerdown模式
	byte dat[2]={0x20,0x0f};
	err=I2C0_MasterSendBlock(I2C0DeviceDataPtr,dat,2,LDD_I2C_SEND_STOP);
	delay();
	
	//设置量程为500dps
	byte dat_ref[2]={0x23,0x10};
	err|=I2C0_MasterSendBlock(I2C0DeviceDataPtr,dat_ref,2,LDD_I2C_SEND_STOP);
	delay();
	return err;
}

LDD_TError read_reg(byte dat_reg,byte num,byte *dat)
{   
	CS_SetVal();
	LDD_TError err;
     LDD_I2C_TBusState I2C0_Busstate;
    dat_reg|=0x80;
   // I2C0_SelectSlaveDevice(I2C0DeviceDataPtr,LDD_I2C_ADDRTYPE_7BITS,0x68);
    
    do
    {
    err=I2C0_MasterSendBlock(I2C0DeviceDataPtr,&dat_reg,(LDD_I2C_TSize)0x01u,LDD_I2C_NO_SEND_STOP);
	//while(!I2C0_send);
	delay();
	
	err|=I2C0_MasterReceiveBlock(I2C0DeviceDataPtr,dat,num,LDD_I2C_SEND_STOP);
	}while(err!=0);
    do
    {I2C0_CheckBus(I2C0DeviceDataPtr,&I2C0_Busstate);}
    while(I2C0_Busstate!=LDD_I2C_IDLE);
    CS_ClrVal();
    
	return err;
}

LDD_TError getdps(float *dps_x,float *dps_y,float *dps_z)
{
	 LDD_TError err;
	 short dps;
	 word dat;
     byte out[8];
	 err=read_reg(0x26,8,out);
	  
	 dat=(word)out[3];
	 dat=(dat<<8)|(word)out[2];
	 dps=(short)dat;
	 *dps_x=(float)dps*(-1);
	 *dps_x=(*dps_x)*17.5/1000;
	  
	 dat=(word)out[5];
	 dat=(dat<<8)|(word)out[4];
	 dps=(short)dat;
	 *dps_y=(float)dps;
	 *dps_y=(*dps_y)*17.5/1000;
	  
	 dat=(word)out[7];
	 dat=(dat<<8)|(word)out[6];
	 dps=(short)dat;
	 *dps_z=(float)dps;
	 *dps_z=(*dps_z)*17.5/1000; 
	 return err;
}
/*
LDD_TError read_z_Gpyo(float *dps_z)
{
	 LDD_TError err; 
	 byte out[2];
	 short dps;
	 word dat;
	 err=read_reg(0x2c,2,out);
	 dat=(word)out[1];
	 dat=(dat<<8)|(word)out[0];
	 dps=(short)dat;
	*dps_z=(float)dps;
		 *dps_z=(*dps_z)*17.5/1000; 
	 return err;
}*/
////////////////////////////////////////

LDD_TError getdps_x(float *dps_x)
{
	 LDD_TError err;
	 short dps;
	 word dat;
     byte out[2];
	 err=read_reg(0x28,2,out);
	  
	 dat=(word)out[1];
	 dat=(dat<<8)|(word)out[0];
	 dps=(short)dat;
	 *dps_x=(float)(dps);
	 *dps_x=(*dps_x)*0.0175-gyro_x_bias;
	 //arm_sub_f32(dps_x,&sub,dps_x,1);
	 return err;
}
LDD_TError getdps_y(float *dps_y)
{
	 LDD_TError err;
	 short dps;
	 word dat;
     byte out[2];
	 err=read_reg(0x2a,2,out);
	  
	 dat=(word)out[1];
	 dat=(dat<<8)|(word)out[0];
	 dps=(short)dat;
	 *dps_y=(float)dps;
	 *dps_y=(*dps_y)*0.0175-gyro_y_bias;
	 return err;
}

LDD_TError getdps_z(float *dps_z)
{
	 LDD_TError err;
	 short dps;
	 word dat;
     byte out[2];
	 err=read_reg(0x2c,2,out);
	  
	 dat=(word)out[1];
	 dat=(dat<<8)|(word)out[0];
	 dps=(short)dat;
	 *dps_z=(float)dps;
	 *dps_z=(*dps_z)*0.0175-gyro_z_bias;
	 return err;
}











