/*
 * Get_speed_frequence.c
 *
 *  Created on: Feb 18, 2014
 *      Author: Administrator
 */
#include "Get_speed_frequence.h"
#include "Cpu.h"
#include "Events.h"
#include "I2C0.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
#include "Bit1.h"
#include "BitIoLdd1.h"
#include "FTM0_PWM_CTL_Motor.h"
#include "FTM1_capture_pfre.h"
#include "LPTMR0_capture_fre.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include"global.h"
#include <math.h>
#include "arm_math.h"
//   0.151371 mm/pulse
LDD_TDeviceData* FTM1_DeviceData=NULL;
LDD_TError       FTM1_Error;
LDD_TDeviceData* LFTM_DeviceData=NULL;
LDD_TError       LFTM1_Error;
 float pulse_length=8.301;		//mm/pulse  1200mm/7228pulse*50
//frequence_value0;
//frequence_value1;

void Init_FTM1_amd_LPTM0()
{
	LFTM_DeviceData=LPTMR0_capture_fre_Init(NULL);
    LPTMR0_capture_fre_Enable(LFTM_DeviceData);
	FTM1_DeviceData=FTM1_capture_pfre_Init(NULL);
	FTM1_Error=FTM1_capture_pfre_Enable(FTM1_DeviceData);
	
	PORTA_PCR19|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTC_PCR5|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
}
LDD_TError  Get_frequence(FTM1_capture_pfre_TValueType *value0,FTM1_capture_pfre_TValueType *value1)
{
	if(Get_frequence_finish_flag)
	{
		*value0=frequence_value0;
		*value1=frequence_value1;
		Get_frequence_finish_flag=FALSE;
		return ERR_OK;
	}
	else return ERR_SPEED;
}

LDD_TError FTM1_LPTMR0_reset(void)
{    
	LDD_TError Err;
	Err=FTM1_capture_pfre_ResetCounter(FTM1_DeviceData);
	Err|=LPTMR0_capture_fre_ResetCounter(LFTM_DeviceData);
	return Err;
}

void FTM1_LPTMR0_getcountervalue(FTM1_capture_pfre_TValueType *ftm1_value,FTM1_capture_pfre_TValueType *lftm_value)
{
	*lftm_value=LPTMR0_capture_fre_GetCounterValue(LFTM_DeviceData);
	*ftm1_value=FTM1_capture_pfre_GetCounterValue(FTM1_DeviceData);
}
/*******************************************************
 *当前的速度（1S内的走过的距离）
 * Car_cal_speed   为正：向前
 * 				          为负：向后
 * 
 *当前两轮的速度差（1S内的距离之差）				          
 *Dif_speed		          为正 ：向右转
 *				          为负：向左转	          				    
 ******************************************************/
float left_speed,right_speed;
void Get_speed_and_DIR(float *Car_cal_speed,float*Dif_speed,
		FTM1_capture_pfre_TValueType value_right,
		FTM1_capture_pfre_TValueType value_left)
{
//	float my_dif_speed;
//	float float_cal_speed;
	bool DIR_right;
	bool DIR_left;
	
	DIR_left=left_Wheel_DIR_GetVal();
	DIR_right=Right_Wheel_DIR_GetVal();
	left_speed=(float)value_left*pulse_length;
	right_speed=(float)value_right*pulse_length;
	if(DIR_right==0)   left_speed=(-1)*left_speed;       //    反方向转动
	if(DIR_left==1)   right_speed=(-1)*right_speed;	    //   反方向转动
	
	*Dif_speed=right_speed-left_speed;				//计算转速差     右轮的脉冲减去左轮的脉冲
	*Car_cal_speed=(left_speed+right_speed)*0.5;       //计算车速
	
//	arm_mult_f32(&float_cal_speed,&pulse_length,Car_cal_speed,0x01);
//	arm_mult_f32(&my_dif_speed,&pulse_length,Dif_speed,0x01);
}







