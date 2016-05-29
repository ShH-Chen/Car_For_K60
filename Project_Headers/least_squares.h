/*
 * least_squares.h
 *
 *  Created on: Mar 15, 2014
 *      Author: Administrator
 */

#ifndef LEAST_SQUARES_H_
#define LEAST_SQUARES_H_

#include "global.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "math.h"
#include "arm_math.h"
#include "stdlib.h"

float sum(float *x, uint32_t n);
float _2_sum(float *x, uint32_t n);
void test();
float B_value(float *x_squ,float *y_squ, byte blockSize,float *a);
#endif /* LEAST_SQUARES_H_ */
