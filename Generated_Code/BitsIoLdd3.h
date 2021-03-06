/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : BitsIoLdd3.h
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
**          Component name                                 : BitsIoLdd3
**          Port                                           : PTE
**          Pins                                           : 4
**            Pin0                                         : 
**              Pin                                        : ADC0_SE17/EXTAL1/PTE24/CAN1_TX/UART4_TX/I2S1_TX_FS/EWM_OUT_b/I2S1_RXD1
**              Pin signal                                 : 
**            Pin1                                         : 
**              Pin                                        : ADC0_SE18/XTAL1/PTE25/CAN1_RX/UART4_RX/I2S1_TX_BCLK/EWM_IN/I2S1_TXD1
**              Pin signal                                 : 
**            Pin2                                         : 
**              Pin                                        : ADC3_SE5b/PTE26/ENET_1588_CLKIN/UART4_CTS_b/I2S1_TXD0/RTC_CLKOUT/USB_CLKIN
**              Pin signal                                 : 
**            Pin3                                         : 
**              Pin                                        : ADC3_SE4b/PTE27/UART4_RTS_b/I2S1_MCLK
**              Pin signal                                 : 
**          Direction                                      : Input
**          Initialization                                 : 
**            Init. direction                              : Input
**            Init. value                                  : 0
**            Auto initialization                          : yes
**          Safe mode                                      : yes
**     Contents    :
**         Init   - LDD_TDeviceData* BitsIoLdd3_Init(LDD_TUserData *UserDataPtr);
**         GetVal - uint32_t BitsIoLdd3_GetVal(LDD_TDeviceData *DeviceDataPtr);
**         GetBit - LDD_TError BitsIoLdd3_GetBit(LDD_TDeviceData *DeviceDataPtr, uint8_t Bit,...
**
**     (c) 2012 by Freescale
** ###################################################################*/
/*!
** @file BitsIoLdd3.h
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
**  @addtogroup BitsIoLdd3_module BitsIoLdd3 module documentation
**  @{
*/         

#ifndef __BitsIoLdd3_H
#define __BitsIoLdd3_H

/* MODULE BitsIoLdd3. */

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
#define BitsIoLdd3_PRPH_BASE_ADDRESS  0x400FF100U
  
/*! Device data structure pointer used when auto initialization property is enabled. This constant can be passed as a first parameter to all component's methods. */
#define BitsIoLdd3_DeviceData  ((LDD_TDeviceData *)PE_LDD_GetDeviceStructure(PE_LDD_COMPONENT_BitsIoLdd3_ID))

/* Methods configuration constants - generated for all enabled component's methods */
#define BitsIoLdd3_Init_METHOD_ENABLED /*!< Init method of the component BitsIoLdd3 is enabled (generated) */
#define BitsIoLdd3_GetVal_METHOD_ENABLED /*!< GetVal method of the component BitsIoLdd3 is enabled (generated) */
#define BitsIoLdd3_GetBit_METHOD_ENABLED /*!< GetBit method of the component BitsIoLdd3 is enabled (generated) */

/* Definition of implementation constants */
#define BitsIoLdd3_MODULE_BASE_ADDRESS PTE_BASE_PTR /*!< Name of macro used as the base address */
#define BitsIoLdd3_PORTCONTROL_BASE_ADDRESS PORTE_BASE_PTR /*!< Name of macro used as the base address */
#define BitsIoLdd3_PORT_MASK 0x0F000000U /*!< Mask of the allocated pin from the port */
#define BitsIoLdd3_PIN_ALLOC_0_MASK 0x01000000 /*!< Mask of the first allocated pin from the port */
#define BitsIoLdd3_PIN_ALLOC_0_INDEX 24U /*!< The index of the first allocated pin from the port */



/*
** ===================================================================
**     Method      :  BitsIoLdd3_Init (component BitsIO_LDD)
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
LDD_TDeviceData* BitsIoLdd3_Init(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Method      :  BitsIoLdd3_GetVal (component BitsIO_LDD)
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
uint32_t BitsIoLdd3_GetVal(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  BitsIoLdd3_GetBit (component BitsIO_LDD)
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
LDD_TError BitsIoLdd3_GetBit(LDD_TDeviceData *DeviceDataPtr, uint8_t Bit, bool *BitVal);

/* END BitsIoLdd3. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif
/* ifndef __BitsIoLdd3_H */
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
