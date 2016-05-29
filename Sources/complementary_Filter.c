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
 * �����˲�����ʱ�䳣��t=1s
 * ���������Ϊ��dt=0.02s
 * �����ǵĸ�ͨϵ��A_gyro=t/(t+dt)=0.98
 * ���ٶȼƵĵ�ͨϵ��A_acc=1-A_gyro=0.02
 **********************************************/
/**********************************************
tao Խ��Խ���������� ԽСԽ���μ��ٶȼ�
 **********************************************/
//const float A_gyro=0.90;		//�����ǵĸ�ͨϵ��
//const float A_acc=0.10;    		//���ٶȼƵĵ�ͨϵ��
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


