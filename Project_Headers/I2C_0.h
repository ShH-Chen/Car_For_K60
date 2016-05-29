/*
 * I2C_0.h
 *
 *  Created on: Feb 14, 2014
 *      Author: sheng
 */

#ifndef I2C_0_H_
#define I2C_0_H_

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

void Init_I2C0(void);
LDD_TError Init_L3D(void);
LDD_TError read_reg(byte dat_reg,byte num,byte *dat);
LDD_TError getdps(float *dps_x,float *dps_y,float *dps_z);
LDD_TError getdps_z(float *dps_z);
LDD_TError getdps_y(float *dps_y);
LDD_TError getdps_x(float *dps_x);

void SelectSlavetoCamera(void);
LDD_TError ReadCameraReg(byte CameraReg,byte *RegNum);
LDD_TError WriteCameraReg(byte CameraReg,byte RegNum);
#endif /* I2C_0_H_ */
