/* ###################################################################
**     Filename    : ProcessorExpert.c
**     Project     : ProcessorExpert
**     Processor   : MK60DN512VLQ10
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2014-02-23, 14:18, # CodeGen: 101
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file ProcessorExpert.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup ProcessorExpert_module ProcessorExpert module documentation
**  @{
*/         
/* MODULE ProcessorExpert */


/* ###################################################################
**     Filename    : ProcessorExpert.c
**     Project     : ProcessorExpert
**     Processor   : MK60DN512VLQ10
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2014-02-17, 21:15, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file ProcessorExpert.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup ProcessorExpert_module ProcessorExpert module documentation
**  @{
*/         
/* MODULE ProcessorExpert */
/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "I2C0.h"
#include "EIntV.h"
#include "ExtIntLdd1.h"
#include "cam8.h"
#include "BitsIoLdd1.h"
#include "Bit1.h"
#include "BitIoLdd1.h"
#include "standcontrol.h"
#include "FTM0_PWM_CTL_Motor.h"
#include "DMAT1.h"
#include "FTM1_capture_pfre.h"
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
#include "LPTMR0_capture_fre.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
#include "string.h"
#include "stdlib.h"
#include "I2C_0.h"
#include <math.h>
#include "arm_math.h"
#include "Kaerman_Filter.h"
#include "complementary_Filter.h"
#include "PID_stand_upright.h"
#include "global.h"
#include "PWM_CTL_Motre_commmon.h"
#include "Get_speed_frequence.h"
#include "Camera.h"
#include "DMA_transfer.h"
#include"speed_pid.h"
#include "angle_pid.h"
#include"process_Camera_image.h"
#include "OLED_12864.h"
#include "MK60_mcg.h"
#include"Card_SDHC.h"
#include "MMA8451_I2C.h"
#include "key.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
float dps_x,dps_y,dps_z,dps;
float angle,angle_source;
float motor_value=0;
float motor_speed=0;
float angle_duty_last=0;
bool start_flag=FALSE;
extern Inc_PID f_Inc_PID;
extern Inc_PID f_speed_PID;
extern float cal_speed;
extern float dif_speed;
extern float gyro_x_bias;
extern float gyro_y_bias;
extern float gyro_z_bias;


byte send_sign;
bool flag_over=TRUE;
bool Initprocess=FALSE;
float angle_duty=0.0;
float angle_duty_dot=0;
bool blue_receive=FALSE;

bool delay_sign=FALSE;
bool motor_start=FALSE;

float Init_speed;

extern float angle_P ;
extern float angle_I ;
extern float angle_D ;

extern float angle_P2 ;
extern float angle_I2 ;
extern float angle_D2 ;

extern float speed_p1;
extern float speed_i1;
extern road_type roadtype;

extern LDD_TDeviceData* LFTM_DeviceData;
extern LDD_TDeviceData* FTM1_DeviceData;

extern byte edge_image[60][80];
extern float angle_value; //方向值
extern float left_pwm_back,right_pwm_back,middle_pwm;
extern float xielu;
extern float para1;

//extern byte  img_undist[NEDD_ROW+10*2][OV7725_EAGLE_W+10*2];

float battery_voltage;
word ad0_num;
/*lint -save  -e970 Disable MISRA rule (6.3) checking. */

word snt;

void m_delay(word delay_num)
{
	unsigned int i1=1000;
	while(delay_num--)
	{
		i1=1000;
		while(i1--);
	}
}


byte valid;
extern float dps_all;
extern float angle_value_back;
extern float saidao_length,left_speed,right_speed;
extern byte camera_cnt;
extern float error_all;
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{  /* Write your local variable definition here */
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
   m_delay(1000);	
   PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/
    ///////////////////////////////////////////////////////
   /*
   float flash_number1;
   float test_num1=3.5;
   IFsh1_SetBlockFlash((byte*)&test_num1,250*2048,4);
   IFsh1_GetBlockFlash(250*2048,(byte*)&flash_number1,4);*/
    LCD_Init(); //OLED初始化  
    Init_I2C0();                //初始化i2c0
    Error=Init_L3D();
    InitCamera();
    Init_DMA();
    
    Init_MMA8451();
    Init_FTM0();				//初始化PWM控制电机
    Init_FTM1_amd_LPTM0();
    Initcom_filter();
    Init_speedpid();    
    set_speed(0);
    Init_speed=(boma1_GetVal()^0x0f)*200;
    if(Init_speed!=0)  Init_speed=Init_speed+1000;
    Init_angle_PD();            //根据速度初始化转向参数
     /**************************************初始化完毕************************/
    float bias[3];
   /* if(!boma2_GetBit(1))
    { 
       LCD_P8x16Str(0,0,"gyro bias test");
       int gyro_count;float x_all=0,y_all=0,z_all=0;
       for(gyro_count=0;gyro_count<=3000;gyro_count++)
       {
    	 getdps_x(&dps_x);
    	 getdps_y(&dps_y);
    	 getdps_z(&dps_z);
    	 x_all=x_all+dps_x;
    	 y_all=y_all+dps_y;
    	 z_all=z_all+dps_z;
    	 m_delay(1);
       }
       bias[0]=x_all/3000;
       bias[1]=y_all/3000;
       bias[2]=z_all/3000;
       IFsh1_SetBlockFlash((byte*)bias,250*2048,12);
       m_delay(1000);
       LCD_P8x16Str(0,2,"test complete");
       m_delay(1000);
    }*/
    
    IFsh1_GetBlockFlash(250*2048,(byte*)bias,12);
    m_delay(1000); 
    gyro_x_bias=bias[0];
    gyro_y_bias=bias[1];
    gyro_z_bias=bias[2];   
    /***********************************参数设置****************************************/
    para_set();
    para_display();
    /******************************************************/
    Initprocess=TRUE;                        //启动直立控速
    para1=63;
    /*********************延时模块************************/
    delay_ResetCounter(delay_DeviceData);
    delay_TValueType delay_TValue;
    do{delay_TValue=delay_GetCounterValue(delay_DeviceData);}
    while(delay_TValue>(1500000000-(75000000*3)));
    motor_start=TRUE;
    do{delay_TValue=delay_GetCounterValue(delay_DeviceData);}
    while(delay_TValue>(1500000000-(75000000*4)));
    start_flag=TRUE;
	/***************************************************/
    set_speed(Init_speed);
    
    char flag=0;
    float turn_pwm_value;
    
    
     while(1)
       { 
    	 if(flag_over)
    	 {   
    		   if(flag>10)
    		   { 
    			 count_time_SetVal();
    		     Camera_Data_Convert();
    		     count_time_ClrVal();
    		     
    		     flag_over=FALSE;//开始下一场采集
    		     
    		     getdps_x(&dps_x);
    		     getdps_z(&dps_z);
    		     arm_sqrt_f32(dps_x*dps_x+dps_z*dps_z,&dps);
    		     if(dps_x>0)  dps=(-1)*dps;
    		     
    		     get_second_centre(&valid);
    		     angle_duty_last=angle_duty;
    		     turn_pwm_value=angle_PD(valid);
    		     angle_duty=turn_pwm_value;
    		     angle_duty_dot=angle_duty-angle_duty_last;
    		    // angle_duty=-1500;
    		   }
    		    else
    		        flag++;
    	  }

    		 if(!boma2_GetBit(0))
    		 {      		    
    			 image_to_12864();
    		    
    		    //LCD_P8x16Str(80,0, "angle");
    		 /*   if(error_all>=10) error_all=9.99;
    		    display_num(88,0,(byte)(error_all));
    		    LCD_P8x16Str(96,0,".");
    		    display_num(104,0,(byte)(error_all*10)%10);
    		    display_num(112,0,(byte)(error_all*100)%10); */
   			     display_num(80,0,(int)para1%1000/100);
   			     display_num(88,0,((int)para1%100)/10);
   			     display_num(96,0,((int)para1%10));
    		    
    		    if(angle_value<0)
    		    {angle_value=(-1)*angle_value; LCD_P8x16Str(80,2,"-"); }
    		    else LCD_P8x16Str(80,2," ");
    		    display_num(88,2,(byte)(angle_value/10));
    		    display_num(96,2,(byte)((byte)angle_value%10));
    		    LCD_P8x16Str(104,2,".");
    		    display_num(112,2,(byte)(angle_value*10)%10);
    		    
    		    if(roadtype==normal_road) 	           
    		        LCD_P8x16Str(80,4, "normal");
    		    else if(roadtype==cross_road)
    		    	LCD_P8x16Str(80,4, "cross ");
    		    else if(roadtype==left_ren)
    		    	LCD_P8x16Str(80,4, "left_r");
    		    else if(roadtype==right_ren)
    		    	LCD_P8x16Str(80,4, "rightr");
    		    else if(roadtype==podao_road)
    		    	LCD_P8x16Str(80,4, "podao");
    		    else if(roadtype==stop_road)
    		    	LCD_P8x16Str(80,4, "stop");
    		    	           
    		  if(xielu<0)
    		    {xielu=(-1)*xielu; LCD_P8x16Str(80,6,"-"); }
    		    else LCD_P8x16Str(80,6," ");
    		    display_num(88,6,(byte)(xielu));
    		    LCD_P8x16Str(96,6,".");
    		    display_num(104,6,(byte)(xielu*10)%10);
    		    display_num(112,6,(byte)(xielu*100)%10);  	   	  
    		  }
    			while(camera_cnt<5);
    			camera_cnt=0;
    			send_sign=0;
    	}

  /* Write your code here */
  /* For example: for(;;) { } */

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END ProcessorExpert */
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
