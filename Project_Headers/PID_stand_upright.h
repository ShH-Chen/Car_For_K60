/*
 * PID_stand_upright.h
 *
 *  Created on: Feb 16, 2014
 *      Author: Administrator
 */

#ifndef PID_STAND_UPRIGHT_H_
#define PID_STAND_UPRIGHT_H_
#include "Cpu.h"
#include "Events.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#define   bala_angle        -6
/**************************************************************
 * 位置PID   P和D参数
 **************************************************************/
extern float  P_pos_upright ;               //1100
extern float  D_pos_upright	;                    //20

/**************************************************************
 * 增量式PID  PID参数
 *************************************************************/
extern float  P_inc_upright ;
extern float  I_inc_upright ;
extern float  D_inc_upright ;

extern float  P_pos_upright2 ;
extern float  D_pos_upright2 ;




typedef struct 
{
float  SetPoint; 			//设定目标 Desired Value
float  SumError;		 	//误差累计
float  Proportion; 		//比例常数 Proportional Const
float  Integral; 		//积分常数 Integral Const
float  Derivative; 		//微分常数 Derivative Const
float  LastError; 			//Error[-1]
float  PrevError; 			//Error[-2]
} Inc_PID;


typedef struct 
{
	float  SetPoint; 			//设定目标 Desired Value
	float  SumError;		 	//误差累计
	float  A; 		//比例常数 Proportional Const
	float  B; 		//积分常数 Integral Const
	float  C; 		//微分常数 Derivative Const
	float  LastError; 			//Error[-1]
	float  PrevError; 			//Error[-2]	
}Change_Inc_PID;

void Init_Inc_PID();
float  PID_pos_CTL_upright(float z_angle,float z_Gypo);
float  PID_inc_CTL_upright(float z_angle);

#endif /* PID_STAND_UPRIGHT_H_ */
