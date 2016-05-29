/*
 * PID_stand_upright.c
 *
 *  Created on: Feb 16, 2014
 *      Author: Administrator
 */
#include"PID_stand_upright.h"
#include <math.h>
#include "arm_math.h"
#include "Cpu.h"
#include "Events.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"


float  upright_set_value = bala_angle ;//-41.8
 
float P_pos_upright  =  1200;
float D_pos_upright  =	50;

/***********************************************************
 * Œª÷√ ΩPIDÀ„∑®
 ***********************************************************/
float  PID_pos_CTL_upright(float z_angle,float z_Gypo)
{	
	float f_Motoer_value;
	       f_Motoer_value=P_pos_upright*(z_angle-upright_set_value)+D_pos_upright*z_Gypo;
	return f_Motoer_value;
}

