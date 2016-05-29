/*
 * complementary_Filter.h
 *
 *  Created on: Feb 16, 2014
 *      Author: Administrator
 */

#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_


#include "Cpu.h"
#include "Events.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

void Initcom_filter(void);
float com_filter(float Acc_angle,float gyro);

#endif /* COMPLEMENTARY_FILTER_H_ */
