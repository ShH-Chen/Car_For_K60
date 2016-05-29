/*
 * Kaerman_Filter.c
 *
 *  Created on: Feb 15, 2014
 *      Author: Administrator
 */


/*
 * kaerman-filter.c
 *
 *  Created on: Feb 15, 2014
 *      Author: Administrator
 */

/*
 * Kaerman_Filter.c
 *
 *  Created on: Jan 28, 2014
 *      Author: Administrator
 */

#include"Kaerman_Filter.h"

#include "Cpu.h"
#include "Events.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
#include "I2C0.h"
#include "Bit1.h"
#include "BitIoLdd1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "I2C_0.h"


	static   float X01=0,X02=0;				        //上次的最佳估计值
	static   float P01=1,P02=0,P03=0,P04=1;	//上次的最佳估计值
	static   float dt=0.005;					        //采样时间
	static   float Q1=0.0007,Q2=0,Q3=0,Q4=0.005;		
	static   float R=0.5;
	static   float X11=0,X12=0;				        //本次的预测值
	static   float P11=0.0,P12=0.0,P13=0.0,P14=0.0;	//本次的预测值
	static   float K1=0,K2=0;				            //卡尔曼增益

const float kA_angle_valu[10]={0,1.8,3.6,5.4,7.2,9,10,10,10,10};
const float kgyro[10]={90,90,90,90,90,90,0,0,0,0};


float Kaerman_filter(float Acc_angle,float speed_gyro )
{
	
	kaerman my_kaerman_value;
	X11=X01+dt*(speed_gyro-X02);
	X12=X02;						//预测
	
	
	//P11=(float)(P01-dt*(P03+P02)+(dt*dt)*P04+Q1);
	P11=P01+dt*(Q1-P03-P02);
	P12=P02-P04*dt+Q2;
	P13=P03-P04*dt+Q3;
	P14=P04+Q4;						//预测
	/*			预测结束					*/

	/************************************************************************/
	/* 更新开始                                                                     */
	/************************************************************************/

	K1=P11/(P11+R);
	K2=P13/(P13+R);

	X01=X11+K1*(Acc_angle-X11);
	X02=X12+K2*(Acc_angle-X11);

	P01=P11*(1-K1);
	P02=P12*(1-K1);
	P03=P13-P11*K2;
	P04=P14-P12*K2;
	
	
	my_kaerman_value.angle=X01;
	my_kaerman_value.gyro_bias=X02;
	my_kaerman_value.gyro_angle=speed_gyro-X02;
	return  my_kaerman_value.angle;
}
/*
kaerman kaerman_init(float z_angle,float dps_z)
{
	kaerman kaerman_over;
	kaerman_over=Kaerman_filter(z_angle,dps_z );
	return kaerman_over;
}*/
/*
void init_kcom()
{
	kaerman com_angle_filter[10];
	byte i=0;
	for(i=0;i<10;i++)
	{
		com_angle_filter[i]=com_filter(A_angle_valu[i],gyro[i]);
		com_angle_filter[i]= Kaerman_filter(kA_angle_valu[i],kgyro[i]);
	}
	
}
*/
