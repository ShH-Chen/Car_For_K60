/*
 * Kaerman_Filter.h
 *
 *  Created on: Feb 15, 2014
 *      Author: Administrator
 */

#ifndef KAERMAN_FILTER_H_
#define KAERMAN_FILTER_H_

#include "Cpu.h"
#include "Events.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"



















typedef struct
{
	float angle;
	float gyro_bias;
	float gyro_angle;
}kaerman;

float  Kaerman_filter(float Acc_angle,float speed_gyro );
kaerman   kaerman_init(float z_angle,float dps_z);

#endif /* KAERMAN_FILTER_H_ */
