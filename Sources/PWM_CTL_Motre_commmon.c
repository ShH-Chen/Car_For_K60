/*
 * PWM_CTL_Motre_commmon.c
 *
 *  Created on: Feb 18, 2014
 *      Author: Administrator
 */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "FTM0_PWM_CTL_Motor.h"
#include "PWM_CTL_Motre_commmon.h"
#include "math.h"

LDD_TDeviceData *FTM0_DeviceData=NULL;
LDD_TError       FTM0_Error;
void Init_FTM0()
{
	FTM0_DeviceData=FTM0_PWM_CTL_Motor_Init(NULL);
	FTM0_Error=FTM0_PWM_CTL_Motor_Enable(FTM0_DeviceData);
	CTL_right_Motor(0);
	CTL_left_Motor(0);
}
/***************************************************
 * 设置左轮的方向
 * Fornt：向前
 * Back：  向后
 **************************************************/
void  Set_left_direction(Motor_direction  direction)
{
	switch(direction)
		{
		case Fornt:
					{  
					FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId0,OUTPUT_SET,OUTPUT_CLEAR);
					FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId1,OUTPUT_CLEAR,OUTPUT_NONE);
					}break;		   
		case Back:
					{
					FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId1,OUTPUT_SET,OUTPUT_CLEAR);
					FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId0,OUTPUT_CLEAR,OUTPUT_NONE);	
					}break;
		default:break;
		}
}

/***************************************************
 * 设置右轮的方向
 * Fornt：向前
 * Back：  向后
 **************************************************/
void Set_right_direction(Motor_direction  direction)
{
	switch(direction)
	{
	case Fornt:
			{
				FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId2,OUTPUT_SET,OUTPUT_CLEAR);
				FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId3,OUTPUT_CLEAR,OUTPUT_NONE);	
			}break;
	case Back:
			{
				FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId3,OUTPUT_SET,OUTPUT_CLEAR);
				FTM0_PWM_CTL_Motor_SelectOutputAction(FTM0_DeviceData,PWM_ChannelId2,OUTPUT_CLEAR,OUTPUT_NONE);
			}break;
	default:break;
	}	
}

/***************************************************
 * 设置左轮的占空比
 * dutycycle:0-100代表0%-100%
 **************************************************/
LDD_TError Set_left_dutycycle(int dutycycle)
{
	LDD_TError my_error;
	FTM0_PWM_CTL_Motor_TValueType my_Motor_dutyTicks;
	FTM0_PWM_CTL_Motor_TValueType my_Motor_offsetTicks;
	my_error=FTM0_PWM_CTL_Motor_GetPeriodTicks(FTM0_DeviceData,&my_Motor_dutyTicks);
	my_Motor_offsetTicks=my_Motor_dutyTicks*(10000-dutycycle)/10000;
	my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId0,my_Motor_offsetTicks);
	my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId1,my_Motor_offsetTicks);
	return my_error;
}

/***************************************************
 * 设置右轮的占空比
 * dutycycle:0-100代表0%-100%
 **************************************************/
LDD_TError Set_right_dutycycle(int dutycycle)
{
		LDD_TError my_error;
		FTM0_PWM_CTL_Motor_TValueType my_Motor_dutyTicks;
		FTM0_PWM_CTL_Motor_TValueType my_Motor_offsetTicks;
		my_error=FTM0_PWM_CTL_Motor_GetPeriodTicks(FTM0_DeviceData,&my_Motor_dutyTicks);
		my_Motor_offsetTicks=my_Motor_dutyTicks*(10000-dutycycle)/10000;
		my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId2,my_Motor_offsetTicks);
		my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId3,my_Motor_offsetTicks);
		return my_error;
}
/***************************************************
 * 设置左右轮的占空比
 * dutycycle:0-100代表0%-100%
 **************************************************/
LDD_TError Set_common_dutycycle(int dutycycle)
{
		LDD_TError my_error;
		FTM0_PWM_CTL_Motor_TValueType my_Motor_dutyTicks;
		FTM0_PWM_CTL_Motor_TValueType my_Motor_offsetTicks;
		my_error=FTM0_PWM_CTL_Motor_GetPeriodTicks(FTM0_DeviceData,&my_Motor_dutyTicks);
		my_Motor_offsetTicks=my_Motor_dutyTicks*(10000-dutycycle)/10000;
		my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId0,my_Motor_offsetTicks);
		my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId1,my_Motor_offsetTicks);
		my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId2,my_Motor_offsetTicks);
		my_error|=FTM0_PWM_CTL_Motor_SetOffsetTicks(FTM0_DeviceData,PWM_ChannelId3,my_Motor_offsetTicks);
		return my_error;
}
/********************************************************************
 * 设置左轮参数
 * dutycycle:0-100代表0%-100%
 * direction：方向
 *******************************************************************/
LDD_TError CTL_left_Motor(int dutycycle)
{   

	if(dutycycle>10000)   dutycycle=10000;
    if(dutycycle<-10000)  dutycycle=-10000;
    //if((dutycycle<850)&&(dutycycle>-850)) dutycycle=0;
	if(dutycycle<0)
    { Set_left_direction(Back);
      dutycycle=(-1)*dutycycle;
	  return Set_left_dutycycle(dutycycle);
    }
    else
    { Set_left_direction(Fornt);
	  return Set_left_dutycycle(dutycycle);}
}
/********************************************************************
 * 设置右轮参数
 * dutycycle:0-100代表0%-100%
 * direction：方向
 *******************************************************************/
LDD_TError CTL_right_Motor(int dutycycle)
{   

	if(dutycycle>10000)   dutycycle=10000;
    if(dutycycle<-10000)  dutycycle=-10000; 
//    if((dutycycle<850)&&(dutycycle>-850)) dutycycle=0; 
	if(dutycycle<0)
     { 
	   Set_right_direction(Fornt);
	   return Set_right_dutycycle((-1)*dutycycle);
     }
     else
     {
    	 Set_right_direction(Back);
    	 return Set_right_dutycycle(dutycycle);
     }
}

















