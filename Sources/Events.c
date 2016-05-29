/* ###################################################################
**     Filename    : Events.c
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
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"

#ifdef __cplusplus
extern "C" {
#endif 

#include "stdlib.h"
#include "string.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "I2C_0.h"
#include"global.h"
#include"Get_speed_frequence.h"
#include "arm_math.h"
#include "Kaerman_Filter.h"
#include "complementary_Filter.h"
#include "PID_stand_upright.h"
#include "global.h"
#include "PWM_CTL_Motre_commmon.h"
#include "Get_speed_frequence.h"
#include "Camera.h"
#include "FTM1_capture_pfre.h"
#include "LPTMR0_capture_fre.h"
#include "left_Wheel_DIR.h"
#include "Right_Wheel_DIR.h"
#include "Get_speed_frequence.h"
#include"speed_pid.h"
#include"Card_SDHC.h"
#include "MMA8451_I2C.h"
#include "process_Camera_image.h"
#include "GPIO_I2C.h"
/* User includes (#include below this line is not maintained by Processor Expert) */

extern  LDD_TDeviceData *DMA_DeviceData;
extern  LDD_TError       DMA__TError;
extern  byte Dest_data[max_hang][max_lie];
static	bool  V_value_flag=FALSE;
extern  bool  flag_over;
extern  LDD_TDeviceData* FTM1_DeviceData;
extern  LDD_TDeviceData* LFTM_DeviceData;
extern byte send_sign;

extern float dps_y;
extern float angle,angle_source;
extern float motor_value;
extern float motor_speed;
float motor_speed_old=0;
extern bool Initprocess;
LPTMR0_capture_fre_TValueType LPTMR0_value;
FTM1_capture_pfre_TValueType  FTM1_value;
float cal_speed=0;
float dif_speed=0;
extern Inc_PID f_Inc_PID;
extern float angle_duty;
extern float angle_duty_last;
extern float angle_duty_dot;
extern bool TX_flag;
extern bool motor_start;
extern bool delay_sign;

extern bool Inserted_flag;
extern bool Removed_flag;
extern bool Finished_flag;
extern TSDData  SD;
extern float dps;
extern byte key_sel;
extern byte key_sel2;
extern byte key_sel3;
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
void Cpu_OnNMIINT(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  bluetooth_OnRxChar (module Events)
**
**     Component   :  bluetooth [AsynchroSerial]
**     Description :
**         This event is called after a correct character is received.
**         The event is available only when the <Interrupt
**         service/event> property is enabled and either the <Receiver>
**         property is enabled or the <SCI output mode> property (if
**         supported) is set to Single-wire mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
extern bool blue_receive;
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
void EIntV_OnInterrupt(void)
{
  /* Write your code here ... */
  if(Initprocess==TRUE)
  { 
	if(!flag_over)
	{
			  if(V_value_flag)
			  {  
				  DMAT1_SetDestinationAddress(DMA_DeviceData,(LDD_DMA_TAddress)&Dest_data[0][0]);
				  PORTC_PCR6|=PORT_PCR_IRQC(2);	
				  V_value_flag=FALSE;  
			  }
			  else
			  {
			      PORTC_PCR6&=~PORT_PCR_IRQC(2);
				  V_value_flag=TRUE;
				  flag_over=TRUE;
			  }	
	}
  }
}

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
float test_pwm_left,test_pwm_right;
float Right_PWM=0;
float left_PWM=0;
float Right_PWMlast=0;
float left_PWMlast=0;
float left_pwm_back,right_pwm_back,middle_pwm;
float speed_all=0;
float saidao_length=0;
float dps_all=0;
float dpsx_bias=-0.095;
byte camera_cnt=0;
extern road_type roadtype;
void standcontrol_OnCounterRestart(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	float pwmturn;
	camera_cnt++;
	if(camera_cnt>5)  camera_cnt=5;
	static char speed_flag=0;
	if(Initprocess==TRUE)
	{  
		static word num_timer=0;
	    num_timer++;
	  if(num_timer==5)
	   {
		    num_timer=0;
		    if(num_timer==1200)   
		    	FTM1_LPTMR0_reset();
		    LPTMR0_value=LPTMR0_capture_fre_GetCounterValue(LFTM_DeviceData);
		    FTM1_value=FTM1_capture_pfre_GetCounterValue(FTM1_DeviceData);
		    Get_speed_and_DIR(&cal_speed,&dif_speed,LPTMR0_value,FTM1_value);
		    FTM1_LPTMR0_reset();
		    speed_all+=cal_speed;
		    if(speed_flag==4)
		    {
		       speed_flag=0;
		       motor_speed_old=motor_speed;
		       motor_speed=PID_speed(speed_all/4);
		       speed_all=0;
		    }
		    speed_flag++;
	    }
		/***********************Ö±Á¢¿ØÖÆ******************************/
	  get_acc(&angle_source);
      getdps_y(&dps_y);
     // dps_x=dps_x-dpsx_bias;
      dps_all=dps_all+dps_y*0.004;
	  angle=com_filter(angle_source,dps_y);
  //    angle=Kaerman_filter(angle_source,dps_x );
	  motor_value=PID_pos_CTL_upright(angle,dps_y);
//	 Right_PWM=motor_value;
	 Right_PWM=motor_value-motor_speed_old-( (motor_speed-motor_speed_old)*0.25)*((int)speed_flag+1);
	 left_PWM = Right_PWM;
	 middle_pwm=left_PWM;
  
	 pwmturn=angle_duty_last+angle_duty_dot/5*camera_cnt;
/*	 if(pwmturn>1000) { Right_PWM-=1000;left_PWM+=pwmturn-1000;}
	 else if(pwmturn<-1000) { Right_PWM-=angle_duty+1000;pwmturn+=-1000;	}
	 else*/
	 { Right_PWM-=pwmturn;left_PWM+=pwmturn;	 }
//	 if(abs(Right_PWMlast-Right_PWM)>2000)    Right_PWM=Right_PWMlast;
//	 if(abs(left_PWMlast-left_PWM)>2000)      left_PWM=left_PWMlast;
//	 Right_PWMlast=Right_PWM;
//	 left_PWMlast=left_PWM;
	 if(motor_start==FALSE)
	 {
		 Right_PWM=0;
		 left_PWM=0;
	 }
	 if(FTM1_value>1200||LPTMR0_value>1200)  {Cpu_SystemReset();}
	 left_pwm_back=left_PWM;
	 right_pwm_back=Right_PWM;
	CTL_left_Motor((int)left_PWM);
	  CTL_right_Motor((int)Right_PWM);
//	  CTL_left_Motor((int)test_pwm_left);
//      CTL_right_Motor((int)test_pwm_right);
	}
		/**************************************************************/
	////////////////////////////////////////////////////////////////
}

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
void camera_loop_OnCounterRestart(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	send_sign=1;
}


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
void SDHC_OnCardInserted(LDD_TUserData *UserDataPtr, uint8_t Id)
{
  /* Write your code here ... */
	Inserted_flag=TRUE;
		TSDData *SD = (TSDData*)UserDataPtr;
		SD->CardId = Id;
}

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
void SDHC_OnCardRemoved(LDD_TUserData *UserDataPtr, uint8_t Id)
{
  /* Write your code here ... */
	TSDData *SD = (TSDData*)UserDataPtr;
	SD->CardId = SDHC_NO_CARD;
	Removed_flag=TRUE;
}

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
void SDHC_OnFinished(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	TSDData *SD = (TSDData*)UserDataPtr;
	Finished_flag=TRUE;
	SD->Finished = TRUE;
}

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
extern float para1;
extern float para2;
extern float para3;
void left_button_OnInterrupt(void)
{
  /* Write your code here ... */
	if(key_sel2==3)  key_sel2=0;
	else key_sel2++;
}

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
void center_button_OnInterrupt(void)
{
  /* Write your code here ... */
	if(key_sel3==0)  key_sel3=1;
	else key_sel3=0;
}

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
void up_button_OnInterrupt(void)
{
  /* Write your code here ... */
	if(key_sel3==0)
	{
		if(key_sel==0) key_sel=3;
			   else key_sel--;
	}
	else
	{
		if(key_sel2==0)  para1=para1+0.1;
		else  if(key_sel2==1)  para1=para1+1;
		else  if(key_sel2==2)  para1=para1+10;
		else  if(key_sel2==3)  para1=para1+100;
	}
	if(Initprocess==TRUE&&(!boma2_GetBit(0)))
	{
		para1++;
		SCCB_WriteByte(0x9C, (byte)para1);
	}
}

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
void down_button_OnInterrupt(void)
{
  /* Write your code here ... */
	if(key_sel3==0)
	{ 
		if(key_sel==3)  key_sel=0;
	         else key_sel++;
	}
	else
	{
		if(key_sel2==0)  para1=para1-0.1;
		else  if(key_sel2==1)  para1=para1-1;
		else  if(key_sel2==2)  para1=para1-10;
		else  if(key_sel2==3)  para1=para1-100;
	}
	if(Initprocess==TRUE&&(!boma2_GetBit(0)))
	{
		para1--;
		SCCB_WriteByte(0x9C, (byte)para1);
	}
}

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
void right_button_OnInterrupt(void)
{
  /* Write your code here ... */
	if(key_sel2==0) key_sel2=3;
	else key_sel2--;
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

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
