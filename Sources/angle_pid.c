/*
 * angle_pid.c
 *
 *  Created on: Mar 4, 2014
 *      Author: sheng
 */


#include "process_Camera_image.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "PID_stand_upright.h"
#include "arm_math.h"
#include "buzzer.h"
#define gpro_bias  0.288
#define P_C	 0	


float angle_para=300;
float ren_para_length;
float ren_para_pwm_l;
float ren_para_pwm_r;
byte ren_done=0;
float ren_length=0;
extern float cal_speed;
float zhidao_speed;

extern float dif_speed;
extern float dps;
extern float angle;
extern float carry_centre[NEED_ROW];
extern byte  Get_extract_image[OV7725_EAGLE_H][OV7725_EAGLE_W];
extern float  upright_set_value;
extern byte left_guaidian1;
extern byte right_guaidian1;

float hang_value_24[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,5,5,5,
    5,5,1,1,1,1,1,0.9,0.8,0.5,
    0.4,0.3,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0};



float hang_value_16[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0};


float hang_value_26[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,5,5,5,5,5,
    0.9,0.8,0.5,0.4,0.3,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,};

float hang_value_28[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,5,5,5,5,5,
    0.9,0.8,0.5,0.4,0.3,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,};

float hang_value_30[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,5,5,5,5,5,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,};

float hang_value_32[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,5,5,5,5,5,
    0.9,0.8,0.5,0.4,0.3,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,};

float hang_value_34[60]=
 {  0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,5,
    5,5,5,5,1,1,1,0.9,0.8,0.5,
    0.4,0.3,0,0,0,0,0};

float* hang_value;
float hang_value_u[60];
float carry_row[60]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15.16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,
                     31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,
                     51,52,53,54,55,56,57,58,59,60};
float k1=150;
float k2=150;
float k3=150;
float k4=200;
float k5=200;
float offset_P=150;	//50
float offset_D=20;	//300/

float K_xielu=0;//2500
float D_xielu=0;//2000
float KP=0;//600
float KD=0;//11
float angle_D=0;//500

float K_middle=0;
float Key_KD=0;
float angle_value,last_angle_value;
float xielu,xielu_last=0;
extern float  Init_speed;
extern road_type roadtype;
extern bool motor_start;
extern byte ren_hang;
extern byte zhangai_left,zhangai_right;
float angle_value_back;
float angle_return_last;
float offset,offset_old;
byte ren_turnl;byte ren_turnr;
byte cross_flag=0;
float error_all=0;
signed char zhidao_flag;
byte zhangai_flag=0;
float zhangai_count=0;
signed char jiaozheng=0;
signed char jiaozhengkey=0;
word qipao_count=0;
extern float saidao_length;
extern Road_Conditon My_Road_Condition[NEED_ROW];
byte podao_flag=0;
float podao_count;

void Init_angle_PD(void)
{  
	if(Init_speed==1600)
	{
		 K_xielu=0;
		 D_xielu=0;
		 KP=140;
		 Key_KD=10;
		 angle_D=200;
		 ren_para_pwm_l=2200;
		 ren_para_pwm_r=1800;
		 ren_para_length=600;
		 hang_value=hang_value_16;
	}
	else if(Init_speed==2000)
	{
		 K_xielu=0;
		 D_xielu=0;
		 KP=150;
		 Key_KD=8;
		 angle_D=200;
		 ren_para_pwm_l=2200;
		 ren_para_pwm_r=1800;
		 ren_para_length=600;
		 hang_value=hang_value_24;
	}
	else if(Init_speed==2200)
	{
		 K_xielu=0;
		 D_xielu=0;
		 KP=160;
		 Key_KD=10;
		 angle_D=200;
		 ren_para_pwm_l=2200;
		 ren_para_pwm_r=2150;
		 ren_para_length=600;
		 hang_value=hang_value_24;
	}
    else if(Init_speed==2400)
	{
		 K_xielu=0;
		 D_xielu=0;
		 K_middle=150;
		 Key_KD=7;
		 angle_D=100;
		 ren_para_pwm_l=2700;
		 ren_para_pwm_r=2300;
		 ren_para_length=600;
		 hang_value=hang_value_24;
		 jiaozheng=0;
	}
    else if(Init_speed==2600)
	{
		 K_xielu=200;
		 D_xielu=0;
		 K_middle=160;
		 KP=110;
		 Key_KD=10;//10
		 angle_D=200;
		 ren_para_pwm_l=2800;
		 ren_para_pwm_r=2200;
		 ren_para_length=610;
		 hang_value=hang_value_26;
	}
	else if(Init_speed==2800)
	{
		 K_xielu=200;
		 D_xielu=0;
		 K_middle=170;
		 KP=110;
		 Key_KD=10;//10
		 angle_D=200;
		 ren_para_pwm_l=2900;
		 ren_para_pwm_r=2200;
		 ren_para_length=610;
		 hang_value=hang_value_28;
	}
	else if(Init_speed==3000)
	{
		 K_xielu=200;
		 D_xielu=0;
		 K_middle=190;
		 KP=110;
		 Key_KD=10;//10
		 angle_D=200;
		 ren_para_pwm_l=3100;
		 ren_para_pwm_r=2400;
		 ren_para_length=610;
		 hang_value=hang_value_30;
	}
	else if(Init_speed==3200)
	{
		 K_xielu=200;
		 D_xielu=0;
		 K_middle=200;
		 KP=110;
		 Key_KD=10;//10
		 angle_D=200;
		 ren_para_pwm_l=3300;
		 ren_para_pwm_r=3300;
		 ren_para_length=610;
		 hang_value=hang_value_32;
	}
	else if(Init_speed==3400)
	{
		 K_xielu=0;
		 D_xielu=0;
		 K_middle=110;
		 KP=135;
		 Key_KD=8;
		 angle_D=0;
		 ren_para_pwm_l=3200;
		 ren_para_pwm_r=3200;
		 ren_para_length=600;
		 hang_value=hang_value_34;
	}
	zhidao_speed=Init_speed;
}

float angle_PD(float valid1)
{  
	 static float podao_num=0;   
    float angle_return_value;
    float chushu=0;
    float angle_dot,xielu_dot;
    float diuxian_value;
    signed int counterx;
    
    saidao_length=saidao_length+cal_speed*0.02;
    if(saidao_length>200000)   saidao_length=200000;
//    if(saidao_length>30000)   motor_start=FALSE;
       
    if(valid1<55&&valid1>=0)
       diuxian_value=carry_centre[(byte)(58-valid1)];
/*    if(angle>0)  
    	buzzer_ClrVal();
    else 
    	buzzer_SetVal();*/
    if(Init_speed==2000)
    {
    	jiaozheng=0;
    	jiaozheng=jiaozheng+jiaozhengkey-1;
    }
    else if(Init_speed==2200)
    {
    	jiaozheng=0;
    	jiaozheng=jiaozheng+jiaozhengkey+3;
    }
    else if(Init_speed==2400)
    {   KP=K_middle;
        jiaozheng=0;
    	jiaozheng=jiaozheng+jiaozhengkey+3;}
    else if(Init_speed==2600)
    {
    	KP=K_middle;
    	jiaozheng=(cal_speed-2400)/70;
    	if(jiaozheng>0)  jiaozheng=0;
    	if(jiaozheng<-6) jiaozheng=-6;
    	jiaozheng=jiaozheng+jiaozhengkey-2;
    }
    else if(Init_speed==2800)
    {
    	KP=K_middle;
    	KP=K_middle+(cal_speed-2400)/70;
    	jiaozheng=(cal_speed-2400)/70;
    	if(jiaozheng>3)  jiaozheng=3;
    	if(jiaozheng<-3) jiaozheng=-3;
    	jiaozheng=jiaozheng+jiaozhengkey;
    }
    else if(Init_speed==3000)
    {
    	KP=K_middle;
    	KP=K_middle+(cal_speed-2400)/60;
    	if(KP>K_middle+10) KP=K_middle+10;
    	if(KP<K_middle-10) KP=K_middle-10;
    	jiaozheng=(cal_speed-2400)/70;
    	if(jiaozheng>6)  jiaozheng=6;
    	if(jiaozheng<0) jiaozheng=0;
    	jiaozheng=jiaozheng+jiaozhengkey;
    }
    else if(Init_speed==3200)
     {
    	KP=K_middle;
    	KP=K_middle+(cal_speed-2500)/60;
    	if(KP>K_middle+10) KP=K_middle+10;
    	if(KP<K_middle-10) KP=K_middle-10;
    	jiaozheng=(cal_speed-2500)/70;
    	if(jiaozheng>6)  jiaozheng=6;
    	if(jiaozheng<0) jiaozheng=0;
    	jiaozheng=jiaozheng+jiaozhengkey+3;
     }
    else if(Init_speed==3400)
     {
    	KP=K_middle+(cal_speed-2800)/35;
    	if(KP<K_middle-35)  KP=K_middle-35;
    	if(KP>K_middle+5) KP=K_middle;
    	K_xielu=KP*9;
    	if(angle>=12) 
    		jiaozheng=4;
    	else if(angle>=15) jiaozheng=6;
    	else if(angle>=10) jiaozheng=2;
    	else if(angle<=7)  jiaozheng=-3;
    	else jiaozheng=0;
    	jiaozheng=jiaozheng+jiaozhengkey;
     }
       
       if(jiaozheng>0) 
       {
    	   for(counterx=0;counterx<=jiaozheng;counterx++)
    	   {
    		   hang_value_u[counterx]=0;
    	   }
    	   for(counterx=jiaozheng;counterx<=59;counterx++)
    	   {
    		   hang_value_u[counterx]=hang_value[counterx-jiaozheng];
    	   }
       }
       else 
       {
    	   for(counterx=0;counterx<=59+jiaozheng;counterx++)
    	   {
    		   hang_value_u[counterx]=hang_value[counterx+abs(jiaozheng)];
    	   }
    	   for(counterx=59+jiaozheng;counterx<=59;counterx++)
    	   {
    		   hang_value_u[counterx]=0;
    	   }
       }
       angle_value=0;        
       for(counterx=0;counterx<59-valid1;counterx++)
        {
    	    angle_value=angle_value+hang_value_u[counterx]*(carry_centre[counterx]-VIDEO_CENTER);
    	    chushu=chushu+hang_value_u[counterx];
        }
       for(counterx=59-valid1;counterx<=59;counterx++)
       {
    	 angle_value=angle_value+(diuxian_value-VIDEO_CENTER)*hang_value_u[counterx];
    	 chushu=chushu+hang_value_u[counterx];
       }
     if(chushu!=0)   angle_value=angle_value/chushu;
   //   angle_value=angle_value+4;
   //   if(angle_value>35) angle_value=35;

      
    if(valid1<55)
          xielu=B_value(carry_row,carry_centre,(byte)(60-valid1),&offset);
    
/**********************************Ö±µÀ¼ì²â*************************************************/
    error_all=0;
    for(counterx=0;counterx<59-valid1;counterx++)
    {
    	error_all=error_all+abs(carry_centre[counterx]-(offset+xielu*(counterx+1)));
    }
    error_all=error_all/(59-valid1);
    if((error_all<1)&&(xielu<0.15)&&(xielu>-0.15)&&(valid1<4))  {zhidao_flag++;}
    else {zhidao_flag=0;}
    if(zhidao_flag>20) zhidao_flag=20;
    if(zhidao_flag==0)  set_speed(Init_speed);
    else set_speed(zhidao_speed);
/*    if(podao_flag==0)
            set_speed(Init_speed+zhidao_flag*20);
    else set_speed(Init_speed);*/
  /***********************************************************************************************/  
    if(zhangai_left&&xielu<0.4&&xielu>-0.4)  {angle_value=angle_value+6;zhangai_flag=1;zhangai_count=0;}
    else if(zhangai_right&&xielu<0.4&&xielu>-0.4) {angle_value=angle_value-6;zhangai_flag=1;zhangai_count=0;}
    else {}
    if(zhangai_flag==1) zhangai_count=zhangai_count+cal_speed*0.02;
    if(zhangai_count>1300)   {zhangai_count=0;zhangai_flag=0;}
/*******************************************************************************************/
//    if(zhidao_flag==1)  set_speed(2800);
//    else set_speed(Init_speed);
    /******************************************************************************/
 //   angle_value=turn_hang*xielu+offset;
    offset=offset-40;
    angle_value_back=angle_value;
    angle_dot=angle_value-last_angle_value;
    xielu_dot=xielu-xielu_last;
    
    
    angle_return_value=KP*angle_value+KD*dps+angle_D*angle_dot+
                             K_xielu*xielu+D_xielu*xielu_dot;
    
    //angle_return_value=K_xielu*xielu+dps*KD+offset_P*offset+offset_D*(offset-offset_old);
    if(angle_return_value>0)  angle_return_value=angle_return_value*0.93;
    /********************************************************************************************/
    static word cross_count=0;
    if(roadtype==cross_road&&cross_count>30&&cross_flag==0)   { cross_flag=1;cross_count=0;}
    cross_count++;
    if(cross_count>60000)  cross_count=60000;
    if(roadtype==cross_road&&cross_count>30&&cross_flag==1)   {cross_flag=0;cross_count=0;}
    if(cross_count>100&&cross_flag==1)   {cross_flag=0;cross_count=0;}
    if(cross_flag==1&&Init_speed>=3000)  KD=Key_KD+1;
    else KD=Key_KD;
    /*********************************************************************************************/
    
    offset_old=offset;
    last_angle_value=angle_value;
    xielu_last=xielu;
    
/*    if(angle_return_value-angle_return_last>500&&angle_return_value>1000&&angle_return_last>0)
    	angle_return_value=angle_return_last+800;
    if(angle_return_value-angle_return_last<-500&&angle_return_value<-1000&&angle_return_last<0) 
    	angle_return_value=angle_return_last-800;*/
//    if(abs(angle_return_value)<500)   angle_return_value=0;
//    if(angle_return_value>4500) angle_return_value=4500;
//   if(angle_return_value<-4500) angle_return_value=-4500;
    angle_return_last=angle_return_value;
//    if(valid1<10&&abs(angle_value)<5)   set_speed(Init_speed+400);
//    else set_speed(Init_speed);
    /*************************************ÆÂµÀ¼ì²â******************************************************/
    
    podao_count=0;byte my_row; 
    for(my_row=NEED_ROW-1;my_row>valid1&&my_row>25;my_row--)
    {
         podao_count=podao_count+My_Road_Condition[my_row].R_position-My_Road_Condition[my_row].L_position;
    }
    if(valid1<25)  podao_count=podao_count/35;
    else podao_count=podao_count/(60-valid1);
    float normal_wide;
    normal_wide=-0.003349*angle*angle+0.8976*angle+48.21;
    if(podao_count<normal_wide-8&&(error_all<2)&&xielu<0.15&&xielu>-0.15&&(valid1<30)&&
    		My_Road_Condition[55].R_position!=COLUMN-1&&My_Road_Condition[55].L_position!=left_lie)   
    {roadtype=podao_road;podao_flag=1;podao_num=1700;}
    if(podao_count>normal_wide+8&&zhidao_flag!=0&&angle>5&&left_guaidian1==0&&right_guaidian1==0)   {roadtype=podao_road;podao_flag=1;podao_num=0;}
    if(podao_count>normal_wide+8&&(error_all<1)&&(xielu<0.15)&&(xielu>-0.15)&&(valid1<30)&&angle<5&&right_guaidian1==0&&left_guaidian1==0)   {roadtype=podao_road;podao_flag=1;podao_num=0;}
    if(podao_num>3000)  {podao_flag=0;podao_num=0;}
    if(podao_flag==1&&boma2_GetBit(2))    
    {
    	podao_num=podao_num+cal_speed*0.02;
    	buzzer_ClrVal();
//    	upright_set_value = bala_angle+12;
    }
    else  
    {
    	buzzer_SetVal();
    }
    /********************************************************************************************/
   static float ren_count=0;
   if(roadtype==left_ren||roadtype==right_ren) buzzer_ClrVal();
   else buzzer_SetVal();
   if(zhangai_flag==0&&Init_speed<=2400&&saidao_length>ren_length*1000)
   { 
     if(roadtype==left_ren&&ren_hang>45&&valid1>10)   { ren_turnl=1;}
     if(roadtype==right_ren&&ren_hang>45&&valid1>10)  { ren_turnr=1;}
     if(ren_count>=120)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
 	 if(ren_turnl==1)  
 	 {
 		if(ren_count<=50) {angle_return_value=0-ren_para_pwm_l;}
 		else angle_return_value=angle_return_value*1.5;
 		ren_count=ren_count+abs(dps)*0.02;
 	 }
 	 else if(ren_turnr==1)   
     {
    	 if(ren_count<=60) angle_return_value=ren_para_pwm_r;
    	 else angle_return_value=angle_return_value*1.5;
    	 ren_count=ren_count+abs(dps)*0.02;
     }
   }
   else if(zhangai_flag==0&&Init_speed==2400&&saidao_length>ren_length*1000)
    { 
      if(roadtype==left_ren&&ren_hang>30&&valid1>10)   { ren_turnl=1;}
      if(roadtype==right_ren&&ren_hang>30&&valid1>10)  { ren_turnr=1;}
      if(ren_count>=ren_para_length)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
      if(ren_turnl==1)   {angle_return_value=0-ren_para_pwm_l;ren_count=ren_count+cal_speed*0.02;}
      if(ren_turnr==1)   {angle_return_value=ren_para_pwm_r;ren_count=ren_count+cal_speed*0.02;}
    }
    else if(zhangai_flag==0&&Init_speed==2600&&saidao_length>ren_length*1000)
    { 
      if(roadtype==left_ren&&ren_hang>25)   { ren_turnl=1;}
      if(roadtype==right_ren&&ren_hang>20)  { ren_turnr=1;}
      if(ren_count>=100)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
  	  if(ren_turnl==1)  
  	  {
  		if(ren_count<=50) {angle_return_value=0-ren_para_pwm_l;}
  		else angle_return_value=angle_return_value*2;
  		ren_count=ren_count+abs(dps)*0.02;
  	  }
  	  else if(ren_turnr==1)   
      {
     	 if(ren_count<=100) angle_return_value=ren_para_pwm_r;
     	 else {angle_return_value=angle_value*100;}
     	 ren_count=ren_count+abs(dps)*0.02;
      }
    }
    else if(zhangai_flag==0&&Init_speed==2800&&saidao_length>ren_length*1000)
    {
    	if(roadtype==left_ren&&ren_hang>25&&valid1>10)   { ren_turnl=1;}
    	if(roadtype==right_ren&&ren_hang>25&&valid1>10)  { ren_turnr=1;}
    	if(ren_count>=50)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
    	if(ren_turnl==1)   {angle_return_value=0-ren_para_pwm_l;ren_count=ren_count+abs(dps)*0.02;}
    	if(ren_turnr==1)   {angle_return_value=ren_para_pwm_r;ren_count=ren_count+abs(dps)*0.02;}
    }
    else if(zhangai_flag==0&&Init_speed==3000&&saidao_length>ren_length*1000)
    {
    	if(roadtype==left_ren&&ren_hang>20&&valid1>10)   { ren_turnl=1;}
    	if(roadtype==right_ren&&ren_hang>20&&valid1>10)  { ren_turnr=1;}
    	if(ren_count>=50)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
    	if(ren_turnl==1)  
    	{
    		angle_return_value=0-ren_para_pwm_l;ren_count=ren_count+abs(dps)*0.02;
    	}
    	if(ren_turnr==1)   
    	{
    		angle_return_value=ren_para_pwm_r;ren_count=ren_count+abs(dps)*0.02;
    	}
    }
    else if(zhangai_flag==0&&Init_speed==3200&&saidao_length>ren_length*1000)
    {
    	if(roadtype==left_ren&&ren_hang>20&&valid1>10)   { ren_turnl=1;}
    	if(roadtype==right_ren&&ren_hang>20&&valid1>10)  { ren_turnr=1;}
    	if(ren_count>=50)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
    	if(ren_turnl==1)   {angle_return_value=0-ren_para_pwm_l;ren_count=ren_count+abs(dps)*0.02;}
    	if(ren_turnr==1)   {angle_return_value=ren_para_pwm_r;ren_count=ren_count+abs(dps)*0.02;}
    }
    else if(zhangai_flag==0&&Init_speed==3400&&saidao_length>ren_length*1000)
    {
    	if(roadtype==left_ren&&ren_hang>20&&valid1>10)   { ren_turnl=1;}
    	if(roadtype==right_ren&&ren_hang>20&&valid1>10)  { ren_turnr=1;}
    	if(ren_count>=ren_para_length)   {ren_turnl=0;ren_turnr=0;ren_count=0;ren_done++;}
    	if(ren_turnl==1)   {angle_return_value=0-ren_para_pwm_l;ren_count=ren_count+cal_speed*0.02;}
    	if(ren_turnr==1)   {angle_return_value=ren_para_pwm_r;ren_count=ren_count+cal_speed*0.02;}
    }
    /*********************************************************************************************/
    if(roadtype==stop_road&&saidao_length>5000&&ren_turnl!=1&&ren_turnr!=1&&boma2_GetBit(3))    
        {motor_start=FALSE;}
    else {;}
    return angle_return_value;
}






