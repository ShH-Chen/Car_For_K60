/*
 * least_squares.c
 *
 *  Created on: Mar 15, 2014
 *      Author: Administrator
 */

#include"least_squares.h"
#include"process_Camera_image.h"
#include "global.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "math.h"
#include "arm_math.h"
#include "stdlib.h"
/***************************************
 * 计算和
 **************************************/
float sum(float *x, uint32_t n)
{
	byte i;
	float add_sum=0;
	for(i=0;i<n;i++,x++)				//计算x*y的和
		{
			arm_add_f32(x,&add_sum,&add_sum,1);
		}
	return add_sum;
}
/********************************************
 * 计算平方和
 *******************************************/
float _2_sum(float *x, uint32_t n)
{
	float x_2[NEED_ROW];
	arm_mult_f32(x,x,x_2,n);
	return sum(x_2,n);
}


float B_value(float *x_squ,float *y_squ, byte blockSize,float *a)
{
	float x_ave;
	float y_ave;
	float XxY[NEED_ROW];
	float xy_sum;
	float x_sum;
	float y_sum;
	static float x_sumXy_sum;
	float x_2_sum;
	float x_sum_2=0;
	float B;
	float A;
	byte i;
	arm_mean_f32 (x_squ,blockSize, &x_ave);				//计算x的平均值
	arm_mean_f32 (y_squ,blockSize,&y_ave);
	arm_mult_f32(x_squ,y_squ, XxY,blockSize);		//计算x*y
	xy_sum=sum(XxY,blockSize);				//计算x*y的和
	x_sum=sum(x_squ,blockSize);
	y_sum=sum(y_squ,blockSize);
	
	
	arm_mult_f32(&x_sum,&y_sum,&x_sumXy_sum,1);
	x_2_sum=_2_sum(x_squ,blockSize);
	for(i=0;i<blockSize;i++)
	{
		arm_add_f32(&x_sum_2,x_squ+i,&x_sum_2,1);
	}
	arm_mult_f32(&x_sum_2,&x_sum_2,&x_sum_2,1);	//计算x和的平方
	B=(blockSize*xy_sum-x_sumXy_sum)/(blockSize*x_2_sum-x_sum_2);
	A=y_ave-B*x_ave;
	*a=A;
	return B;
}

/*
void test()
{	
	float B;
	float x[30]={1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,
				16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0};
	byte j=0;
	for(j=0;j<30;j++)
		x[j]=x[j]/8;
	float y[30]={2.0,4.0,6.0,8.0,10.0,12.0,14.0,16.0,18.0,20.0,22.0,24.0,26.0,28.0,30.0,
			32.0,34.0,36.0,38.0,40.0,42.0,44.0,46.0,48.0,50.0,52.0,54.0,56.0,58.0,60.0};
	
	B=B_value(x,y,30);

}
*/
