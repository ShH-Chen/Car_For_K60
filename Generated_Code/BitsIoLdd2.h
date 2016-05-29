/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : BitsIoLdd2.h
**     Project     : ProcessorExpert
**     Processor   : MK60FX512VLQ15
**     Component   : BitsIO_LDD
**     Version     : Component 01.029, Driver 01.05, CPU db: 3.00.000
**     Compiler    : GNU C Compiler
**     Date/Time   : 2014-07-15, 17:57, # CodeGen: 103
**     Abstract    :
**         The HAL BitsIO component provides a low level API for unified
**         access to general purpose digital input/output 32 pins across
**         various device designs.
**
**         RTOS drivers using HAL BitsIO API are simpler and more
**         portable to various microprocessors.
**     Settings    :
**          Component name                                 : BitsIoLdd2
**          Port                                           : PTA
**          Pins                                           : 4
**            Pin0                                         : 
**              Pin                                        : ADC0_SE11/PTA8/ULPI_NXT/FTM1_CH0/I2S1_RX_FS/FTM1_QD_PHA/TRACE_D2
**              Pin signal                                 : 
**            Pin1                                         : 
**              Pin                                        : ADC3_SE5a/PTA9/ULPI_STP/FTM1_CH1/MII0_RXD3/FTM1_QD_PHB/TRACE_D1
**              Pin signal                                 : 
**            Pin2                                         : 
**              Pin                                        : ADC3_SE4a/PTA10/ULPI_DATA0/FTM2_CH0/MII0_RXD2/FTM2_QD_PHA/TRACE_D0
**              Pin signal                                 : 
**            Pin3                                         : 
**              Pin                                        : ADC3_SE15/PTA11/ULPI_DATA1/FTM2_CH1/MII0_RXCLK/FTM2_QD_PHB
**              Pin signal                                 : 
**          Direction                                      : Input
**          Initialization                                 : 
**            Init. direction                              : Input
**            Init. value                                  : 0
**            Auto initialization                          : yes
**          Safe mode                                      : no
**     Contents    :
**         Init   - LDD_TDeviceData* BitsIoLdd2_Init(LDD_TUserData *UserDataPtr);
**         GetVal - uint32_t BitsIoLdd2_GetVal(LDD_TDeviceData *DeviceDataPtr);
**         GetBit - LDD_TError BitsIoLdd2_GetBit(LDD_TDeviceData *DeviceDataPtr, uint8_t Bit,...
**
**     (c) 2012 by Freescale
** ###################################################################*/
/*!
** @file BitsIoLdd2.h
** @version 01.05
** @brief
**         The HAL BitsIO component provides a low level API for unified
**         access to general purpose digital input/output 32 pins across
**         various device designs.
**
**         RTOS drivers using HAL BitsIO API are simpler and more
**         portable to various microprocessors.
*/         
/*!
**  @addtogroup BitsIoLdd2_module BitsIoLdd2 module documentation
**  @{
*/         

#ifndef __BitsIoLdd2_H
#define __BitsIoLdd2_H

/* MODULE BitsIoLdd2. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */
#include "IO_Map.h"
#include "GPIO_PDD.h"

#include "Cpu.h"

#ifdef __cplusplus
extern "C" {
#endif 



/*! Peripheral base address of a device allocated by the component. This constant can be used directly in PDD macros. */
#define BitsIoLdd2_PRPH_BASE_ADDRESS  0x400FF000U
  
/*! Device data structure pointer used when auto initialization property is enabled. This constant can be passed as a first parameter to all component's methods. */
#define BitsIoLdd2_DeviceData  ((LDD_TDeviceData *)PE_LDD_GetDeviceStructure(PE_LDD_COMPONENT_BitsIoLdd2_ID))

/* Methods configuration constants - generated for all enabled component's methods */
#define BitsIoLdd2_Init_METHOD_ENABLED /*!< Init method of the component BitsIoLdd2 is enabled (generated) */
#define BitsIoLdd2_GetVal_METHOD_ENABLED /*!< GetVal method of the component BitsIoLdd2 is enabled (generated) */
#define BitsIoLdd2_GetBit_METHOD_ENABLED /*!< GetBit method of the component BitsIoLdd2 is enabled (generated) */

/* Definition of implementation constants */
#define BitsIoLdd2_MODULE_BASE_ADDRESS PTA_BASE_PTR /*!< Name of macro used as the base address */
#define BitsIoLdd2_PORTCONTROL_BASE_ADDRESS PORTA_BASE_PTR /*!< Name of macro used as the base address */
#define BitsIoLdd2_PORT_MASK 0x0F00U   /*!< Mask of the allocated pin from the port */
#define BitsIoLdd2_PIN_ALLOC_0_MASK 0x0100 /*!< Mask of the first allocated pin from the port */
#define BitsIoLdd2_PIN_ALLOC_0_INDEX 8U /*!< The index of the first allocated pin from the port */



/*
** ===================================================================
**     Method      :  BitsIoLdd2_Init (component BitsIO_LDD)
*/
/*!
**     @brief
**         This method initializes the associated peripheral(s) and the
**         component internal variables. The method is called
**         automatically as a part of the application initialization
**         code.
**     @param
**         UserDataPtr     - Pointer to the RTOS device
**                           structure. This pointer will be passed to
**                           all events as parameter.
**     @return
**                         - Pointer to the dynamically allocated private
**                           structure or NULL if there was an error.
*/
/* ===================================================================*/
LDD_TDeviceData* BitsIoLdd2_Init(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Method      :  BitsIoLdd2_GetVal (component BitsIO_LDD)
*/
/*!
**     @brief
**         Returns the value of the Input/Output component. If the
**         direction is [input] then reads the input value of the pins
**         and returns it. If the direction is [output] then returns
**         the last written value (see [Safe mode] property for
**         limitations).
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Input value
*/
/* ===================================================================*/
uint32_t BitsIoLdd2_GetVal(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  BitsIoLdd2_GetBit (component BitsIO_LDD)
*/
/*!
**     @brief
**         Returns the value of the specified bit/pin of the
**         Input/Output component. If the direction is [input] then it
**         reads the input value of the pin and returns it. If the
**         direction is [output] then it returns the last written value
**         (see [Safe mode] property for limitations).
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         Bit             - Bit/pin number to read
**     @param
**         BitVal          - The returned value: 
**                           [false] - logical "0" (Low level)
**                           [true] - logical "1" (High level)
**     @return
**                         - Error code, possible values:
**                           ERR_OK - OK
**                           ERR_PARAM_INDEX - Invalid pin index
**                           ERR_PARAM_VALUE - Invalid output parameter
*/
/* ===================================================================*/
LDD_TError BitsIoLdd2_GetBit(LDD_TDeviceData *DeviceDataPtr, uint8_t Bit, bool *BitVal);

/* END BitsIoLdd2. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif
/* ifndef __BitsIoLdd2_H */
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
