/*
 * Get_speed_frequence.h
 *
 *  Created on: Feb 18, 2014
 *      Author: Administrator
 */

#ifndef GET_SPEED_FREQUENCE_H_
#define GET_SPEED_FREQUENCE_H_
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



//extern  LDD_TDeviceData *FTM1_DeviceData;
extern  LDD_TError       FTM1_Error;
//extern  LDD_TDeviceData* LFTM_DeviceData;
extern  LDD_TError       LFTM1_Error;

void Init_FTM1_amd_LPTM0();
LDD_TError  Get_frequence(FTM1_capture_pfre_TValueType *value0,FTM1_capture_pfre_TValueType *value1);
void FTM1_LPTMR0_getcountervalue(FTM1_capture_pfre_TValueType *ftm1_value,FTM1_capture_pfre_TValueType *lftm_value);
LDD_TError FTM1_LPTMR0_reset(void);
void Get_speed_and_DIR(float *Car_cal_speed,float*Dif_speed,
		FTM1_capture_pfre_TValueType value_right,
		FTM1_capture_pfre_TValueType value_left);

#endif /* GET_SPEED_FREQUENCE_H_ */
