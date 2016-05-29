/*
 * Camera.c
 *
 *  Created on: Feb 18, 2014
 *      Author: sheng
 */
#include "Cpu.h"
#include "Events.h"
#include "I2C0.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
#include "Bit1.h"
#include "BitIoLdd1.h"
/* Including shared modules, which are used for whole project */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "global.h"
#include "I2C_0.h"
#include "Camera.h"
#include "FIRE_OV7725_REG.h"
#include "GPIO_I2C.h"
/********************************************************************************/

LDD_TDeviceData* DMA_devicedata;

typedef struct 
{
	byte address;
	byte reg_val;
} reg_s;

/*在 ov7725.h 里，搜索“VSTRT”，在数组 ov7727_reg 里定义的，
     修改 VSTRT 的变量，增大 VSTRT 就是往右移动图片，减少VSTRT 就是往左移动图片。*/

reg_s ov7725_eagle_reg[] =
{
	    //寄存器，寄存器值次
	    {OV7725_COM4         , 0xC1},
	    {OV7725_CLKRC        , 0x02},
	    {OV7725_COM2         , 0x03},
	    {OV7725_COM3         , 0xD0},
	    {OV7725_COM7         , 0x40},
	    {OV7725_HSTART       , 0x3F},
	    {OV7725_HSIZE        , 0x50},
	    
	    {OV7725_VSTRT        , 0x03},
	    
	    {OV7725_VSIZE        , 0x78},
	    {OV7725_HREF         , 0x00},
	    {OV7725_SCAL0        , 0x0A},
	    {OV7725_AWB_Ctrl0    , 0xE0},
	    {OV7725_DSPAuto      , 0xff},
	    {OV7725_DSP_Ctrl2    , 0x0C},
	    {OV7725_DSP_Ctrl3    , 0x00},
	    {OV7725_DSP_Ctrl4    , 0x00},
	    
#if (max_lie == 10)
    {OV7725_HOutSize     , 0x14},
#elif (max_lie == 20)
    {OV7725_HOutSize     , 0x28},
#elif (max_lie == 30)
    {OV7725_HOutSize     , 0x3c},
#elif (max_lie == 40)
    {OV7725_HOutSize     , 0x50},
#else
#endif
#if (max_hang == 60 )
    {OV7725_VOutSize     , 0x1E},
#elif (max_hang == 120 )
    {OV7725_VOutSize     , 0x3c},
#elif (max_hang == 180 )
    {OV7725_VOutSize     , 0x5a},
#elif (max_hang == 240 )
    {OV7725_VOutSize     , 0x78},
#else
#endif

    {OV7725_EXHCH        , 0x00},
    {OV7725_GAM1         , 0x0c},
    {OV7725_GAM2         , 0x16},
    {OV7725_GAM3         , 0x2a},
    {OV7725_GAM4         , 0x4e},
    {OV7725_GAM5         , 0x61},
    {OV7725_GAM6         , 0x6f},
    {OV7725_GAM7         , 0x7b},
    {OV7725_GAM8         , 0x86},
    {OV7725_GAM9         , 0x8e},
    {OV7725_GAM10        , 0x97},
    {OV7725_GAM11        , 0xa4},
    {OV7725_GAM12        , 0xaf},
    {OV7725_GAM13        , 0xc5},
    {OV7725_GAM14        , 0xd7},
    {OV7725_GAM15        , 0xe8},
    {OV7725_SLOP         , 0x20},
    {OV7725_LC_RADI      , 0x00},
    {OV7725_LC_COEF      , 0x13},
    {OV7725_LC_XC        , 0x08},
    {OV7725_LC_COEFB     , 0x14},
    {OV7725_LC_COEFR     , 0x17},
    {OV7725_LC_CTR       , 0x05},
    {OV7725_BDBase       , 0x99},
    {OV7725_BDMStep      , 0x03},
    {OV7725_SDE          , 0x04},
    {OV7725_BRIGHT       , 0x00},
    {OV7725_CNST         , 63},
    {OV7725_SIGN         , 0x06},
    {OV7725_UVADJ0       , 0x11},
    {OV7725_UVADJ1       , 0x02},

};

uint8 ov7725_eagle_cfgnum = ARR_SIZE( ov7725_eagle_reg ) ; /*结构体数组成员数目*/

/*******************************************************************************/
/*
50帧：
{COM4         , 0xC1},
{CLKRC        , 0x02},
75帧：
{COM4         , 0x41},
{CLKRC        , 0x00},
112帧：
{COM4         , 0x81},
{CLKRC        , 0x00},
150帧：
{COM4         , 0xC1},
{CLKRC        , 0x00},
*/

void InitCamera(void)
{   word con,coum;
	//SelectSlavetoCamera();
	//WriteCameraReg(OV7725_COM7,0x80);
     PORTE_PCR10|=PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
    for(con=0;con<ov7725_eagle_cfgnum;con++)
	 {
    
    	SCCB_WriteByte(ov7725_eagle_reg[con].address, ov7725_eagle_reg[con].reg_val);
    	//ReadCameraReg(ov7725_eagle_reg[con].address,&num);
    	//while(ov7725_eagle_reg[con].reg_val!=num);
	 }
    for(coum=0;coum<=40;coum++)  
      for(con=0;con<65535;con++) {;}
    ov7725_eagle_reg[0].reg_val=0x81;
    ov7725_eagle_reg[1].reg_val=0x00;
    
    for(con=0;con<ov7725_eagle_cfgnum;con++)
	 {
    	SCCB_WriteByte(ov7725_eagle_reg[con].address, ov7725_eagle_reg[con].reg_val);
    	//ReadCameraReg(ov7725_eagle_reg[con].address,&num);
    	//while(ov7725_eagle_reg[con].reg_val!=num);
	 }
    
}









