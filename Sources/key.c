/*
 * key.c
 *
 *  Created on: Jul 15, 2014
 *      Author: sheng
 */


#include "OLED_12864.h"
#include "boma2.h"
#include "math.h"
#include "stdlib.h"
#include "GPIO_I2C.h"
#include "OLED_12864.h"

byte key_sel=3;
byte key_sel2=0;
byte key_sel3=0;
extern signed char jiaozhengkey;
extern float ren_length;
extern float K_xielu;//2500
extern float D_xielu;//2000
extern float KP;//600
extern float Key_KD;//11
extern float angle_D;//500
extern float K_middle;
extern float ren_para_length;
extern float ren_para_pwm_l;
extern float ren_para_pwm_r;
extern float zhidao_speed;
extern byte ren_num;

float para1,para;

void para_set(void)
{    
	PORTB_PCR4|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTB_PCR5|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTB_PCR6|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTB_PCR16|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTB_PCR17|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTE_PCR10|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTE_PCR11|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
	PORTE_PCR12|=PORT_PCR_PE_MASK|PORT_PCR_PFE_MASK;
//	if(!boma2_GetBit(3))
    {
    	LCD_P8x16Str(8,0,"jz");
    	LCD_P8x16Str(8,2,"K_m");
    	LCD_P8x16Str(8,4,"KD");
    	LCD_P8x16Str(8,6,"done");
    	while(1)
    	{
    		if(key_sel==0)
    		{
    		  LCD_P8x16Str(0,0,"*");
    		  LCD_P8x16Str(0,2," ");
    		  LCD_P8x16Str(0,4," ");
    		  LCD_P8x16Str(0,6," ");
    		  if(key_sel3==1)
    		  {
    			  jiaozhengkey=para1;
    			  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
    			  else  {LCD_P8x16Str(48,0," ");para=para1;}
    			  display_num(56,0,(int)para%1000/100);
    			  display_num(64,0,((int)para%100)/10);
    			  display_num(72,0,((int)para%10));
    			  LCD_P8x16Str(80,0,".");
    			  display_num(88,0,(int)(para*10)%10);
    			  if(key_sel2==0)      LCD_P8x16Str(56,2,"    -");
    			  else if(key_sel2==1) LCD_P8x16Str(56,2,"  -  ");
    			  else if(key_sel2==2) LCD_P8x16Str(56,2," -   ");
    			  else if(key_sel2==3) LCD_P8x16Str(56,2,"-    ");    				  
    		  }
    		  else para1=jiaozhengkey;
    		}
    		else if(key_sel==1)
    		{
      		  LCD_P8x16Str(0,0," ");
      		  LCD_P8x16Str(0,2,"*");
      		  LCD_P8x16Str(0,4," ");
      		  LCD_P8x16Str(0,6," ");
    		  if(key_sel3==1)
    		  {
    			  K_middle=para1;
    			  if(para1<0)  {LCD_P8x16Str(48,2,"-");para=(-1)*para1;}
    			  else  {LCD_P8x16Str(48,2," ");para=para1;}
    			  display_num(56,2,(int)para%1000/100);
    			  display_num(64,2,((int)para%100)/10);
    			  display_num(72,2,((int)para%10));
    			  LCD_P8x16Str(80,2,".");
    			  display_num(88,2,(int)(para*10)%10);
    			  if(key_sel2==0)      LCD_P8x16Str(56,4,"    -");
    			  else if(key_sel2==1) LCD_P8x16Str(56,4,"  -  ");
    			  else if(key_sel2==2) LCD_P8x16Str(56,4," -   ");
    			  else if(key_sel2==3) LCD_P8x16Str(56,4,"-    ");  
    		  }
    		  else  para1=K_middle;;
    		}
    		else if(key_sel==2)
    		{
      		  LCD_P8x16Str(0,0," ");
      		  LCD_P8x16Str(0,2," ");
      		  LCD_P8x16Str(0,4,"*");
      		  LCD_P8x16Str(0,6," ");
    		  if(key_sel3==1)
    		  {
    			  Key_KD=para1;
    			  if(para1<0)  {LCD_P8x16Str(48,4,"-");para=(-1)*para1;}
    			  else  {LCD_P8x16Str(48,4," ");para=para1;}
    			  display_num(56,4,(int)para%1000/100);
    			  display_num(64,4,((int)para%100)/10);
    			  display_num(72,4,((int)para%10));
    			  LCD_P8x16Str(80,4,".");
    			  display_num(88,4,(int)(para*10)%10);
    			  if(key_sel2==0)      LCD_P8x16Str(56,6,"    -");
    			  else if(key_sel2==1) LCD_P8x16Str(56,6,"  -  ");
    			  else if(key_sel2==2) LCD_P8x16Str(56,6," -   ");
    			  else if(key_sel2==3) LCD_P8x16Str(56,6,"-    ");  
    		  }
    		  else para1=Key_KD;
    		}
    		else if(key_sel==3)
    		{
      		  LCD_P8x16Str(0,0," ");
      		  LCD_P8x16Str(0,2," ");
      		  LCD_P8x16Str(0,4," ");
      		  LCD_P8x16Str(0,6,"*");
      		  if(key_sel3==1) 
      		  { LCD_P8x16Str(56,6,"done...");key_sel3=0; break;}
    		}
    	}
    /***************************************************************************************************/
    	LCD_CLS();
    	LCD_P8x16Str(8,0,"length");
    	LCD_P8x16Str(8,2,"pwml");
    	LCD_P8x16Str(8,4,"R yu");
    	LCD_P8x16Str(8,6,"done");
    	while(1)
    	{
    	    		if(key_sel==0)
    	    		{
    	    		  LCD_P8x16Str(0,0,"*");
    	    		  LCD_P8x16Str(0,2," ");
    	    		  LCD_P8x16Str(0,4," ");
    	    		  LCD_P8x16Str(0,6," ");
    	    		  if(key_sel3==1)
    	    		  {
    	    			  ren_para_length=para1;
    	    			  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
    	    			  else  {LCD_P8x16Str(48,0," ");para=para1;}
    	    			  display_num(56,0,(int)para%10000/1000);
    	    			  display_num(64,0,(int)para%1000/100);
    	    			  display_num(72,0,((int)para%100)/10);
    	    			  display_num(80,0,((int)para%10));
    	    			  LCD_P8x16Str(88,0,".");
    	    			  display_num(92,0,(int)(para*10)%10);
    	    			  if(key_sel2==0)      LCD_P8x16Str(56,2,"     -");
    	    			  else if(key_sel2==1) LCD_P8x16Str(56,2,"   -  ");
    	    			  else if(key_sel2==2) LCD_P8x16Str(56,2,"  -   ");
    	    			  else if(key_sel2==3) LCD_P8x16Str(56,2," -    ");    				  
    	    		  }
    	    		  else para1=ren_para_length;
    	    		}
    	    		else if(key_sel==1)
    	    		{
    	      		  LCD_P8x16Str(0,0," ");
    	      		  LCD_P8x16Str(0,2,"*");
    	      		  LCD_P8x16Str(0,4," ");
    	      		  LCD_P8x16Str(0,6," ");
    	    		  if(key_sel3==1)
    	    		  {
    	    			  ren_para_pwm_l=para1;
    	    			  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
    	    			  else  {LCD_P8x16Str(48,0," ");para=para1;}
    	    			  display_num(56,2,(int)para%10000/1000);
    	    			  display_num(64,2,(int)para%1000/100);
    	    			  display_num(72,2,((int)para%100)/10);
    	    			  display_num(80,2,((int)para%10));
    	    			  LCD_P8x16Str(88,2,".");
    	    			  display_num(92,2,(int)(para*10)%10);
    	    			  if(key_sel2==0)      LCD_P8x16Str(56,4,"     -");
    	    			  else if(key_sel2==1) LCD_P8x16Str(56,4,"   -  ");
    	    			  else if(key_sel2==2) LCD_P8x16Str(56,4,"  -   ");
    	    			  else if(key_sel2==3) LCD_P8x16Str(56,4," -    ");    
    	    		  }
    	    		  else  para1=ren_para_pwm_l;
    	    		}
    	    		else if(key_sel==2)
    	    		{
    	      		  LCD_P8x16Str(0,0," ");
    	      		  LCD_P8x16Str(0,2," ");
    	      		  LCD_P8x16Str(0,4,"*");
    	      		  LCD_P8x16Str(0,6," ");
    	    		  if(key_sel3==1)
    	    		  {
    	    			  ren_length=para1;
    	    			  if(para1<0)  {LCD_P8x16Str(48,4,"-");para=(-1)*para1;}
    	    			  else  {LCD_P8x16Str(48,4," ");para=para1;}
    	    			  display_num(56,4,(int)para%1000/100);
    	    			  display_num(64,4,((int)para%100)/10);
    	    			  display_num(72,4,((int)para%10));
    	    			  LCD_P8x16Str(80,4,".");
    	    			  display_num(88,4,(int)(para*10)%10);
    	    			  if(key_sel2==0)      LCD_P8x16Str(56,6,"    -");
    	    			  else if(key_sel2==1) LCD_P8x16Str(56,6,"  -  ");
    	    			  else if(key_sel2==2) LCD_P8x16Str(56,6," -   ");
    	    			  else if(key_sel2==3) LCD_P8x16Str(56,6,"-    ");  
    	    		  }
    	    		  else para1=ren_length;
    	    		}
    	    		else if(key_sel==3)
    	    		{
    	      		  LCD_P8x16Str(0,0," ");
    	      		  LCD_P8x16Str(0,2," ");
    	      		  LCD_P8x16Str(0,4," ");
    	      		  LCD_P8x16Str(0,6,"*");
    	      		  if(key_sel3==1) 
    	      		  { LCD_P8x16Str(56,6,"done...");key_sel3=0; break;}
    	    		}
    	}
    	/********************************************************************************/
    	LCD_CLS();
    	LCD_P8x16Str(8,0,"pwmr");
    	LCD_P8x16Str(8,2,"zhi_s");
    	LCD_P8x16Str(8,4,"ren_num");
    	LCD_P8x16Str(8,6,"done");
    	while(1)
    	{
    	    		if(key_sel==0)
    	    		{
    	    		  LCD_P8x16Str(0,0,"*");
    	    		  LCD_P8x16Str(0,2," ");
    	    		  LCD_P8x16Str(0,4," ");
    	    		  LCD_P8x16Str(0,6," ");
    	    		  if(key_sel3==1)
    	    		  {
    	    			  ren_para_pwm_r=para1;
    	    			  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
    	    			  else  {LCD_P8x16Str(48,0," ");para=para1;}
    	    			  display_num(56,0,(int)para%10000/1000);
    	    			  display_num(64,0,(int)para%1000/100);
    	    			  display_num(72,0,((int)para%100)/10);
    	    			  display_num(80,0,((int)para%10));
    	    			  LCD_P8x16Str(88,0,".");
    	    			  display_num(92,0,(int)(para*10)%10);
    	    			  if(key_sel2==0)      LCD_P8x16Str(56,2,"     -");
    	    			  else if(key_sel2==1) LCD_P8x16Str(56,2,"   -  ");
    	    			  else if(key_sel2==2) LCD_P8x16Str(56,2,"  -   ");
    	    			  else if(key_sel2==3) LCD_P8x16Str(56,2," -    ");    				  
    	    		  }
    	    		  else para1=ren_para_pwm_r;
    	    		}
    	    		else if(key_sel==1)
    	    		{
    	      		  LCD_P8x16Str(0,0," ");
    	      		  LCD_P8x16Str(0,2,"*");
    	      		  LCD_P8x16Str(0,4," ");
    	      		  LCD_P8x16Str(0,6," ");
    	    		  if(key_sel3==1)
    	    		  {
    	    			  zhidao_speed=para1;
    	    			  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
    	    			  else  {LCD_P8x16Str(48,0," ");para=para1;}
    	    			  display_num(56,2,(int)para%10000/1000);
    	    			  display_num(64,2,(int)para%1000/100);
    	    			  display_num(72,2,((int)para%100)/10);
    	    			  display_num(80,2,((int)para%10));
    	    			  LCD_P8x16Str(88,2,".");
    	    			  display_num(92,2,(int)(para*10)%10);
    	    			  if(key_sel2==0)      LCD_P8x16Str(56,4,"     -");
    	    			  else if(key_sel2==1) LCD_P8x16Str(56,4,"   -  ");
    	    			  else if(key_sel2==2) LCD_P8x16Str(56,4,"  -   ");
    	    			  else if(key_sel2==3) LCD_P8x16Str(56,4," -    ");    
    	    		  }
    	    		  else  para1=zhidao_speed;
    	    		}
    	    		else if(key_sel==2)
    	    		{
    	      		  LCD_P8x16Str(0,0," ");
    	      		  LCD_P8x16Str(0,2," ");
    	      		  LCD_P8x16Str(0,4,"*");
    	      		  LCD_P8x16Str(0,6," ");
    	    		  if(key_sel3==1)
    	    		  {
    	    			  ren_num=para1;
    	    			  if(para1<0)  {LCD_P8x16Str(48,4,"-");para=(-1)*para1;}
    	    			  else  {LCD_P8x16Str(48,4," ");para=para1;}
    	    			  display_num(56,4,(int)para%1000/100);
    	    			  display_num(64,4,((int)para%100)/10);
    	    			  display_num(72,4,((int)para%10));
    	    			  LCD_P8x16Str(80,4,".");
    	    			  display_num(88,4,(int)(para*10)%10);
    	    			  if(key_sel2==0)      LCD_P8x16Str(56,6,"    -");
    	    			  else if(key_sel2==1) LCD_P8x16Str(56,6,"  -  ");
    	    			  else if(key_sel2==2) LCD_P8x16Str(56,6," -   ");
    	    			  else if(key_sel2==3) LCD_P8x16Str(56,6,"-    ");  
    	    		  }
    	    		  else para1=ren_num;
    	    		}
    	    		else if(key_sel==3)
    	    		{
    	      		  LCD_P8x16Str(0,0," ");
    	      		  LCD_P8x16Str(0,2," ");
    	      		  LCD_P8x16Str(0,4," ");
    	      		  LCD_P8x16Str(0,6,"*");
    	      		  if(key_sel3==1) 
    	      		  { LCD_P8x16Str(56,6,"done...");key_sel3=0; break;}
    	    		}
    	}
    }
}

void para_display(void)
{
	LCD_CLS();
	LCD_P8x16Str(8,0,"jz");
	LCD_P8x16Str(8,2,"K_m");
	LCD_P8x16Str(8,4,"KD");
	  para1=jiaozhengkey;
	  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
	  else  {LCD_P8x16Str(48,0," ");para=para1;}
	  display_num(56,0,(int)para%1000/100);
	  display_num(64,0,((int)para%100)/10);
	  display_num(72,0,((int)para%10));
	  LCD_P8x16Str(80,0,".");
	  display_num(88,0,(int)(para*10)%10);
	  
	  para1=K_middle;
	  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
	  else  {LCD_P8x16Str(48,0," ");para=para1;}
	  display_num(56,2,(int)para%1000/100);
	  display_num(64,2,((int)para%100)/10);
	  display_num(72,2,((int)para%10));
	  LCD_P8x16Str(80,2,".");
	  display_num(88,2,(int)(para*10)%10);
	  if(key_sel2==0)      LCD_P8x16Str(56,4,"    -");
	  else if(key_sel2==1) LCD_P8x16Str(56,4,"  -  ");
	  else if(key_sel2==2) LCD_P8x16Str(56,4," -   ");
	  else if(key_sel2==3) LCD_P8x16Str(56,4,"-    ");  
	  
	  para1=Key_KD;
	  if(para1<0)  {LCD_P8x16Str(48,0,"-");para=(-1)*para1;}
	  else  {LCD_P8x16Str(48,0," ");para=para1;}
	  display_num(56,4,(int)para%1000/100);
	  display_num(64,4,((int)para%100)/10);
	  display_num(72,4,((int)para%10));
	  LCD_P8x16Str(80,4,".");
	  display_num(88,4,(int)(para*10)%10);
	  if(key_sel2==0)      LCD_P8x16Str(56,6,"    -");
	  else if(key_sel2==1) LCD_P8x16Str(56,6,"  -  ");
	  else if(key_sel2==2) LCD_P8x16Str(56,6," -   ");
	  else if(key_sel2==3) LCD_P8x16Str(56,6,"-    ");  
}



