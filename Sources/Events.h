/* ###################################################################
**     Filename    : Events.h
**     Project     : ProcessorExpert
**     Processor   : MK60DN512VLQ10
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2014-02-17, 21:15, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Settings    :
**     Contents    :
**         Cpu_OnNMIINT - void Cpu_OnNMIINT(void);
**
** ###################################################################*/
/*!
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "I2C0.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
#include "Bit1.h"
#include "BitIoLdd1.h"
#include "standcontrol.h"
#include "DMA1.h"
#include "PCLR.h"
#include "BitIoLdd2.h"
#include "count_time.h"
#include "BitIoLdd3.h"
#include "camera_loop.h"
#include "left_Wheel_DIR.h"
#include "BitIoLdd4.h"
#include "Right_Wheel_DIR.h"
#include "BitIoLdd5.h"
#include "I2CSCL.h"
#include "BitIoLdd6.h"
#include "I2CSDA.h"
#include "BitIoLdd7.h"
#include "delay.h"
#include "boma1.h"
#include "BitsIoLdd2.h"
#include "SM1.h"
#include "boma2.h"
#include "BitsIoLdd3.h"
#include "SDHC.h"
#include "CS.h"
#include "BitIoLdd10.h"
#include "SDO.h"
#include "BitIoLdd11.h"
#include "MMA8451_SCL.h"
#include "BitIoLdd12.h"
#include "MMA8451_SDA.h"
#include "BitIoLdd13.h"
#include "buzzer.h"
#include "BitIoLdd14.h"
#include "right_button.h"
#include "ExtIntLdd2.h"
#include "down_button.h"
#include "ExtIntLdd3.h"
#include "up_button.h"
#include "ExtIntLdd4.h"
#include "center_button.h"
#include "ExtIntLdd5.h"
#include "left_button.h"
#include "ExtIntLdd6.h"
#include "DC.h"
#include "BitIoLdd8.h"
#include "BitIoLdd9.h"
#include "Reset.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"
#include "FTM1_capture_pfre.h"
#include "LPTMR0_capture_fre.h"
#include "FTM0_PWM_CTL_Motor.h"
#include "DMAT1.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
** ===================================================================
**     Event       :  Cpu_OnNMIINT (module Events)
**
**     Component   :  Cpu [MK60DN512LQ10]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMIINT(void);


/*
** ===================================================================
**     Event       :  I2C0_OnMasterBlockSent (module Events)
**
**     Component   :  I2C0 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C in master mode finishes the
**         transmission of the data successfully. This event is not
**         available for the SLAVE mode and if MasterSendBlock is
**         disabled. 
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void I2C0_OnMasterBlockSent(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Event       :  I2C0_OnMasterBlockReceived (module Events)
**
**     Component   :  I2C0 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C is in master mode and finishes
**         the reception of the data successfully. This event is not
**         available for the SLAVE mode and if MasterReceiveBlock is
**         disabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void I2C0_OnMasterBlockReceived(LDD_TUserData *UserDataPtr);

void EIntH_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  EIntH_OnInterrupt (module Events)
**
**     Component   :  EIntH [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void EIntV_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  EIntV_OnInterrupt (module Events)
**
**     Component   :  EIntV [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/*
** ===================================================================
**     Event       :  standcontrol_OnCounterRestart (module Events)
**
**     Component   :  standcontrol [TimerUnit_LDD]
*/
/*!
**     @brief
**         Called if counter overflow/underflow or counter is
**         reinitialized by modulo or compare register matching.
**         OnCounterRestart event and Timer unit must be enabled. See
**         [SetEventMask] and [GetEventMask] methods. This event is
**         available only if a [Interrupt] is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer passed as
**                           the parameter of Init method.
*/
/* ===================================================================*/
void standcontrol_OnCounterRestart(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Event       :  camera_loop_OnCounterRestart (module Events)
**
**     Component   :  camera_loop [TimerUnit_LDD]
*/
/*!
**     @brief
**         Called if counter overflow/underflow or counter is
**         reinitialized by modulo or compare register matching.
**         OnCounterRestart event and Timer unit must be enabled. See
**         [SetEventMask] and [GetEventMask] methods. This event is
**         available only if a [Interrupt] is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer passed as
**                           the parameter of Init method.
*/
/* ===================================================================*/
void camera_loop_OnCounterRestart(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Event       :  delay_OnCounterRestart (module Events)
**
**     Component   :  delay [TimerUnit_LDD]
*/
/*!
**     @brief
**         Called if counter overflow/underflow or counter is
**         reinitialized by modulo or compare register matching.
**         OnCounterRestart event and Timer unit must be enabled. See
**         [SetEventMask] and [GetEventMask] methods. This event is
**         available only if a [Interrupt] is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer passed as
**                           the parameter of Init method.
*/
/* ===================================================================*/
void delay_OnCounterRestart(LDD_TUserData *UserDataPtr);

void SM1_OnTxChar(void);
/*
** ===================================================================
**     Event       :  SM1_OnTxChar (module Events)
**
**     Component   :  SM1 [SynchroMaster]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/*
** ===================================================================
**     Event       :  SM1_OnBlockSent (module Events)
**
**     Component   :  SM1 [SPIMaster_LDD]
*/
/*!
**     @brief
**         This event is called after the last character from the
**         output buffer is moved to the transmitter. This event is
**         available only if the SendBlock method is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer is passed
**                           as the parameter of Init method. 
*/
/* ===================================================================*/
void SM1_OnBlockSent(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Event       :  SDHC_OnCardInserted (module Events)
**
**     Component   :  SDHC [SDHC_LDD]
*/
/*!
**     @brief
**         This event is called when a card is inserted into the slot.
**         The card identification number is retrieved and it is passed
**         to the event. If card detection pin is not available, the
**         DetectCards method calls this event.
**     @param
**         UserDataPtr     - Pointer to the user data
**                           structure.
**     @param
**         Id              - Card identification number of the inserted
**                           card.
*/
/* ===================================================================*/
void SDHC_OnCardInserted(LDD_TUserData *UserDataPtr, uint8_t Id);

/*
** ===================================================================
**     Event       :  SDHC_OnCardRemoved (module Events)
**
**     Component   :  SDHC [SDHC_LDD]
*/
/*!
**     @brief
**         This event is called after a card is removed. The card
**         identification number is retrieved and it is passed to the
**         event. If card detection pin is not available, the
**         SelectCard method calls this event after a try to select the
**         removed card.
**     @param
**         UserDataPtr     - Pointer to the user data
**                           structure.
**     @param
**         Id              - Card identification number of the removed
**                           card.
*/
/* ===================================================================*/
void SDHC_OnCardRemoved(LDD_TUserData *UserDataPtr, uint8_t Id);

/*
** ===================================================================
**     Event       :  SDHC_OnFinished (module Events)
**
**     Component   :  SDHC [SDHC_LDD]
*/
/*!
**     @brief
**         This event is called after an operation, initiated by user,
**         has finished.
**     @param
**         UserDataPtr     - Pointer to the user data
**                           structure.
*/
/* ===================================================================*/
void SDHC_OnFinished(LDD_TUserData *UserDataPtr);

void left_button_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  left_button_OnInterrupt (module Events)
**
**     Component   :  left_button [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void center_button_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  center_button_OnInterrupt (module Events)
**
**     Component   :  center_button [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void up_button_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  up_button_OnInterrupt (module Events)
**
**     Component   :  up_button [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void down_button_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  down_button_OnInterrupt (module Events)
**
**     Component   :  down_button [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void right_button_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  right_button_OnInterrupt (module Events)
**
**     Component   :  right_button [ExtInt]
**     Description :
**         This event is called when an active signal edge/level has
**         occurred.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.08]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
