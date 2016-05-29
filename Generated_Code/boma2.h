/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : boma2.h
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
**         Port name                   : PTE
**
**         Bit mask of the port        : 0x0F000000
**         Number of bits/pins         : 4
**         Single bit numbers          : 0 to 3
**         Values range                : 0 to 15
**
**         Initial direction           : Input (direction cannot be changed)
**         Initial output value        : 0 = 000H
**         Initial pull option         : off
**
**         Port data register          : GPIOE_PDOR [0x400FF100]
**         Port control register       : GPIOE_PDDR [0x400FF114]
**
**             ----------------------------------------------------
**                   Bit     |   Pin   |   Name
**             ----------------------------------------------------
**                    0      |    45   |   ADC0_SE17/EXTAL1/PTE24/CAN1_TX/UART4_TX/I2S1_TX_FS/EWM_OUT_b/I2S1_RXD1
**                    1      |    46   |   ADC0_SE18/XTAL1/PTE25/CAN1_RX/UART4_RX/I2S1_TX_BCLK/EWM_IN/I2S1_TXD1
**                    2      |    47   |   ADC3_SE5b/PTE26/ENET_1588_CLKIN/UART4_CTS_b/I2S1_TXD0/RTC_CLKOUT/USB_CLKIN
**                    3      |    48   |   ADC3_SE4b/PTE27/UART4_RTS_b/I2S1_MCLK
**             ----------------------------------------------------
**
**         Optimization for            : speed
**     Contents    :
**         GetVal - byte boma2_GetVal(void);
**         GetBit - bool boma2_GetBit(byte Bit);
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
** @file boma2.h
** @version 01.00
** @brief
**         This component "BitsIO" implements a multi-bit input/output.
**         It uses selected pins of one 1-bit to 8-bit port.
**         Note: This component is set to work in Input direction only.
*/         
/*!
**  @addtogroup boma2_module boma2 module documentation
**  @{
*/         

#ifndef __boma2_H
#define __boma2_H

/* MODULE boma2. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */
#include "BitsIoLdd3.h"

#include "Cpu.h"

#ifdef __cplusplus
extern "C" {
#endif 




/*
** ===================================================================
**     Method      :  boma2_GetVal (component BitsIO)
**     Description :
**         This method returns an input value.
**           a) direction = Input  : reads the input value from the
**                                   pins and returns it
**           b) direction = Output : returns the last written value
**         Note: This component is set to work in Input direction only.
**     Parameters  : None
**     Returns     :
**         ---        - Input value (0 to 15)
** ===================================================================
*/
#define boma2_GetVal() ((byte)BitsIoLdd3_GetVal(BitsIoLdd3_DeviceData))

bool boma2_GetBit(byte Bit);
/*
** ===================================================================
**     Method      :  boma2_GetBit (component BitsIO)
**     Description :
**         This method returns the specified bit of the input value.
**           a) direction = Input  : reads the input value from pins
**                                   and returns the specified bit
**           b) direction = Output : returns the specified bit
**                                   of the last written value
**         Note: This component is set to work in Input direction only.
**     Parameters  :
**         NAME       - DESCRIPTION
**         Bit        - Number of the bit to read (0 to 3)
**     Returns     :
**         ---        - Value of the specified bit (FALSE or TRUE)
**                      FALSE = "0" or "Low", TRUE = "1" or "High"
** ===================================================================
*/

/* END boma2. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __boma2_H */
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
