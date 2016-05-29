/*
 * global.h
 *
 *  Created on: Feb 17, 2014
 *      Author: Administrator
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#include "FTM1_capture_pfre.h"
#include "LPTMR0_capture_fre.h"
#include "FTM0_PWM_CTL_Motor.h"

extern byte str[100];
extern char counter;
extern LDD_TError Error;


#define max_lie   10                                                   //摄像头接收数组列
#define max_hang  60                                                   //摄像头接收数组行

extern byte Dest_data[max_hang][max_lie];
#define OV7725_EAGLE_W            80                                   //定义图片解压后图像宽度
#define OV7725_EAGLE_H            60                                    //定义图片解压后图像高度


#ifndef __FTM0__all__value__
#define __FTM0__all__value__
extern FTM1_capture_pfre_TValueType  frequence_value0;
extern FTM1_capture_pfre_TValueType  frequence_value1;
extern bool    Get_frequence_finish_flag;
#endif

#endif /* GLOBAL_H_ */
