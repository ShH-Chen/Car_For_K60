/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : cam8.c
**     Project     : ProcessorExpert
**     Processor   : MK60FX512VLQ15
**     Component   : BitsIO
**     Version     : Component 02.108, Driver 01.00, CPU db: 3.00.000
**     Compiler    : GNU C Compiler
**     Date/Time   : 2014-07-15, 17:57, # CodeGen: 103
**     Abstract    :
**         This component "BitsIO" implements a multi-bit input/output.
**         It uses selected pins of one 1-bit to 8-bit port.
**         Note: This component is set to work in Input direction only.
**     Settings    :
**         Port name                   : PTD
**
**         Bit mask of the port        : 0x00FF
**         Number of bits/pins         : 8
**         Single bit numbers          : 0 to 7
**         Values range                : 0 to 255
**
**         Initial direction           : Input (direction cannot be changed)
**         Initial output value        : 0 = 000H
**         Initial pull option         : off
**
**         Port data register          : GPIOD_PDOR [0x400FF0C0]
**         Port control register       : GPIOD_PDDR [0x400FF0D4]
**
**             ----------------------------------------------------
**                   Bit     |   Pin   |   Name
**             ----------------------------------------------------
**                    0      |    127  |   PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FTM3_CH0/FBa_ALE/FBa_CS1_b/FBa_TS_b/I2S1_RXD1
**                    1      |    128  |   ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/FTM3_CH1/FBa_CS0_b/I2S1_RXD0
**                    2      |    129  |   PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/FTM3_CH2/FBa_AD4/I2S1_RX_FS
**                    3      |    130  |   PTD3/SPI0_SIN/UART2_TX/FTM3_CH3/FBa_AD3/I2S1_RX_BCLK
**                    4      |    131  |   PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FBa_AD2/NFC_DATA1/EWM_IN
**                    5      |    132  |   ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FBa_AD1/NFC_DATA0/EWM_OUT_b
**                    6      |    133  |   ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FBa_AD0/FTM0_FLT0
**                    7      |    136  |   PTD7/CMT_IRO/UART0_TX/FTM0_CH7/FTM0_FLT1
**             ----------------------------------------------------
**
**         Optimization for            : speed
**     Contents    :
**         GetVal - byte cam8_GetVal(void);
**         GetBit - bool cam8_GetBit(byte Bit);
**
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file cam8.c
** @version 01.00
** @brief
**         This component "BitsIO" implements a multi-bit input/output.
**         It uses selected pins of one 1-bit to 8-bit port.
**         Note: This component is set to work in Input direction only.
*/         
/*!
**  @addtogroup cam8_module cam8 module documentation
**  @{
*/         

/* MODULE cam8. */

#include "cam8.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
** ===================================================================
**     Method      :  cam8_GetVal (component BitsIO)
**     Description :
**         This method returns an input value.
**           a) direction = Input  : reads the input value from the
**                                   pins and returns it
**           b) direction = Output : returns the last written value
**         Note: This component is set to work in Input direction only.
**     Parameters  : None
**     Returns     :
**         ---        - Input value (0 to 255)
** ===================================================================
*/
/*
byte cam8_GetVal(void)

**  This method is implemented as a macro. See cam8.h file.  **
*/

/*
** ===================================================================
**     Method      :  cam8_GetBit (component BitsIO)
**     Description :
**         This method returns the specified bit of the input value.
**           a) direction = Input  : reads the input value from pins
**                                   and returns the specified bit
**           b) direction = Output : returns the specified bit
**                                   of the last written value
**         Note: This component is set to work in Input direction only.
**     Parameters  :
**         NAME       - DESCRIPTION
**         Bit        - Number of the bit to read (0 to 7)
**     Returns     :
**         ---        - Value of the specified bit (FALSE or TRUE)
**                      FALSE = "0" or "Low", TRUE = "1" or "High"
** ===================================================================
*/
bool cam8_GetBit(byte Bit)
{
  bool BitVal;

  (void)BitsIoLdd1_GetBit(BitsIoLdd1_DeviceData, Bit, &BitVal);
  return (BitVal);
}

/* END cam8. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
