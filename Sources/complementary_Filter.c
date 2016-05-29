/*
 * complementary_Filter.c
 *
 *  Created on: Feb 16, 2014
 *      Author: Administrator
 */

#include "Cpu.h"
#include "Events.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include  "complementary_Filter.h"
#include "MMA8451_I2C.h"

#define dt 0.004
//float tao=0.7;
/**********************************************
 * 假设滤波器的时间常数t=1s
 * 假设采样率为：dt=0.02s
 * 陀螺仪的高通系数A_gyro=t/(t+dt)=0.98
 * 加速度计的低通系数A_acc=1-A_gyro=0.02
 **********************************************/
/**********************************************
tao 越大越信任陀螺仪 越小越信任加速度计
 **********************************************/
//const float A_gyro=0.90;		//陀螺仪的高通系数
//const float A_acc=0.10;    		//加速度计的低通系数
float A_gyro = 0.995;
float A_acc;

float angle_com=0;


void Initcom_filter(void)
{
	get_acc(&angle_com);
}

float com_filter(float Acc_angle,float gyro)
{   
	//A_gyro=tao/(tao+dt);
    A_acc=1-A_gyro;
	angle_com=A_gyro*(angle_com+gyro*dt)+A_acc*Acc_angle;
	return angle_com;
}


