/*
 * speed_pid.c
 *
 *  Created on: Feb 25, 2014
 *      Author: sheng
 */

#include"speed_pid.h"
#include "arm_math.h"
#include "PWM_CTL_Motre_commmon.h"
#include"PID_stand_upright.h"
#include "stdlib.h"
#include "process_Camera_image.h"

float speed_p3=  15; //3
float speed_i3=  0.7; //0.3

float speed_p2= 20; //5
float speed_i2= 0; //0.3

float speed_p1=  9; //15;
float speed_i1=  0.7; //0.3;
float speed_d1=   0;

float speed_value_duty;
float speed_set_last=0;

Inc_PID f_speed_PID;
bool speed_start_flag=TRUE;

arm_pid_instance_f32 speed;
extern byte ren_turnl,ren_turnr;
extern road_type  roadtype;
extern byte podao_flag;
extern float saidao_length;
extern float Init_speed;
void Init_speedpid(void)
{
	f_speed_PID.Proportion=speed_p2;
	f_speed_PID.Integral=speed_i2;
	f_speed_PID.Derivative=0;
	f_speed_PID.SumError=0;
}

void CTL_Speed(float upright_duty,float Speed_duty,float DIR_duty)
{
	float Right_PWM;
	float left_PWM;
	Right_PWM=upright_duty-Speed_duty+DIR_duty;
	left_PWM=upright_duty-Speed_duty-DIR_duty;
	CTL_left_Motor((int)left_PWM);
	CTL_right_Motor((int)Right_PWM);
}

/*********************************************************
 * 增量式PID控制直立
 * 原始的增量PID算法
 * 参考来自：浅谈增量式PID参数调整
 *********************************************************/
static float speed_sum=0;
float  PID_speed(float speed)
{   
	static char speed_count=0; 
	static float last_speed_sum=0;
	static float last_speed=0;
	float PID_now_error;
	float speed_int,speed_dri;
	if(speed_count==3)
	 {
		if(speed_set_last<f_speed_PID.SetPoint)  speed_set_last=speed_set_last+300;
	    if(speed_set_last>f_speed_PID.SetPoint)  speed_set_last=speed_set_last-300;
	    speed_count=0;
	 }
	speed_count++;
	speed_set_last=f_speed_PID.SetPoint;
	PID_now_error=speed_set_last-speed;
//	if(speed>speed_set_last*0.8&&speed<speed_set_last*1.2)   f_speed_PID.SumError=0;
	if(PID_now_error>7000||PID_now_error<-7000)   PID_now_error=0;
	//if(PID_now_error>600)    PID_now_error=600;
	//if(PID_now_error<-600)   PID_now_error=-600;
//	if(abs(PID_now_error)<500)  
	 {speed_dri=speed_p1*PID_now_error;
	 speed_int=speed_i1*PID_now_error;}
/*	else if(abs(PID_now_error)<1000)  
	{
		speed_dri=speed_p2*PID_now_error;
		speed_int=speed_i2*PID_now_error;
	}
	else   
	{
		speed_dri=speed_p3*PID_now_error;
		speed_int=speed_i3*PID_now_error;
	}*/
	f_speed_PID.SumError+=speed_int;
	if(f_speed_PID.SumError>5000) f_speed_PID.SumError=5000;
	if(f_speed_PID.SumError<-5000) f_speed_PID.SumError=-5000;
	speed_sum=f_speed_PID.SumError+speed_dri+speed_d1*(last_speed-speed);
	if(saidao_length>4000)
	{   
		if(speed_sum-last_speed_sum>5000)   speed_sum=last_speed_sum+5000;
	    if(speed_sum-last_speed_sum<-5000)  speed_sum=last_speed_sum-5000;
	}
	else
	{
		if(speed_sum-last_speed_sum>8000)   speed_sum=last_speed_sum+8000;
	    if(speed_sum-last_speed_sum<-8000)  speed_sum=last_speed_sum-8000;
	}
//	if(ren_turnr||ren_turnl)  return 0;
	if(podao_flag==1&&boma2_GetBit(2))  
	{
		if(last_speed_sum<6000)  last_speed_sum=6000;
		if(last_speed_sum>10000) last_speed_sum=10000;
		return last_speed_sum;
	}
	last_speed=speed;
	last_speed_sum=speed_sum;
	if(speed_sum>25000) speed_sum=25000;
	if(speed_sum<-25000) speed_sum=-25000;
	return (speed_sum);
}

void set_speed(float speed_value)
{
	f_speed_PID.SetPoint=speed_value;
}


