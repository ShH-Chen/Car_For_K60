/*
 * global.c
 *
 *  Created on: Feb 17, 2014
 *      Author: Administrator
 */

#include"global.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#include "FTM1_capture_pfre.h"
#include "LPTMR0_capture_fre.h"
#include "FTM0_PWM_CTL_Motor.h"

byte str[100];
char counter=0;
byte size;
LDD_TError Error;


FTM1_capture_pfre_TValueType  frequence_value0;
FTM1_capture_pfre_TValueType  frequence_value1;
bool    Get_frequence_finish_flag=FALSE;



byte Dest_data[max_hang][max_lie];

