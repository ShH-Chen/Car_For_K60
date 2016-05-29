/*
 * process_Camera_image.c
 *
 *  Created on: Mar 5, 2014
 *      Author: Administrator
 */

#include"process_Camera_image.h"

#include "global.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "math.h"
#include "arm_math.h"
#include "stdlib.h"
#include"least_squares.h"

#define COLUMU_diff_first  8

extern byte ren_done;
/*
const byte Need_Row[60]=
            {
		      20,24,28,32,36,40,44,48,52,55,
		      58,61,64,67,71,75,79,83,87,91,
		      94,97,101,104,107,110,113,116,119,122,
		      124,126,128,130,132,134,136,138,140,142,
		      144,146,148,150,152,154,156,158,160,162,
		      164,166,168,170,172,174,176,178,179,180,
		    };*/
const byte Need_Row[60]=
  {
	  40,41,42,44,46,48,50,52,54,56,
	  58,60,63,66,69,72,75,78,81,84,
	  87,90,93,96,99,102,105,108,111,114,
	  117,120,123,126,129,131,135,139,143,147,
	  151,155,159,163,167,172,177,182,187,192,
	  197,202,207,212,217,222,227,232,237,240,	  
  };
const byte  set_left[NEED_ROW]={ 
		  	  	  	  	  	  	  9, 10,10,11,11,
		  	  	  	  	  	  	  12,12,13,14,14,
		 	 	 	 	 	 	  15,16,16,17,17,
		  	  	  	  	  	  	  18,18,19,19,20,
		  	  	  	  	  	  	  21,21,22,22,23,
		  	  	  	  	  	  	  23,24,24,25,25,
		  	  	  	  	  	  	  26,26,27,27,28,
		  	  	  	  	  	  	  28,29,29,30,30,
		  	  	  	  	  	  	  31,31,32,32,33,
		  	  	  	  	  	  	  34,34,35,35,36,
		  	  	  	  	  	  	  36,37,37,38,38,
		  	  	  	  	  	  	  39,39,41,41,42,

					};
const byte  set_right[NEED_ROW]={
							75,75,74,74,73,
							73,72,72,71,71,
							70,70,69,69,68,
							68,67,67,66,66,
							65,65,64,64,63,
							63,62,62,61,61,
							60,60,59,59,58,
							57,57,56,56,55,
							54,54,53,53,52,
							51,51,50,50,49,
							49,48,48,47,47,
							46,46,45,45,44			
				  };
const byte width[NEED_ROW]=
    {
    		0,0,0,0,0,1,9,10,11,11,
    		12,13,14,14,15,15,16,16,16,17,
    		17,18,20,20,21,21,21,23,23,24,
    		24,25,25,26,26,27,27,27,28,28,
    		29,29,29,30,31,31,31,32,33,33,
    		33,34,35,35,35,35,36,36,37,38
    };

float carry_centre[NEED_ROW];
extern float carry_row[60];


Road_Conditon My_Road_Condition [NEED_ROW];		//每行的标示
byte  Get_extract_image[OV7725_EAGLE_H][OV7725_EAGLE_W];
byte ren_num=2;


byte valid_hang=0;
extern bool start_flag;
float value[60];
extern byte  img_hang[NEED_ROW][OV7725_EAGLE_W];
extern byte  img_lie[NEED_ROW][OV7725_EAGLE_W];
byte last_left_pos;
byte last_right_pos;
/*十字道检测变量*/
byte left_guaidian1=0,left_guaidian2=0,left_guaidian_flag[2];
byte right_guaidian1=0,right_guaidian2=0,right_guaidian_flag[2];
road_type roadtype;
byte danbian_flag=0;byte road_width=0;byte danbian_row=0;
byte ren_left_flag,ren_right_flag,ren_hang;
byte Mid_farCount,L_Count;
byte zhangai_left,zhangai_right;
byte RR_Count;
byte mid_value;

/*
signed int change_w,change_h;
void Camera_Data_Convert(void)
{   int image_h,image_w;
    byte select[2]={0,255};
	for(image_h=0;image_h<OV7725_EAGLE_H;image_h++)
	{	
		for(image_w=0;image_w<OV7725_EAGLE_W;image_w++)
	    {
		   //change_w=(signed int)((float)(image_w-80)*Row_line_value[OV7725_EAGLE_H-image_h]*2+160);
			change_h=Need_Row[image_h];
			//change_w=image_w*3;
			//change_h=image_h*4;
			change_w=image_w*4;
			Get_extract_image[image_h][image_w]=select[(Dest_data[change_h][change_w>>3]>>(7-(change_w&0x07)))&0x01];
	    }
	}
}
*/


void Camera_Data_Convert(void)
{   unsigned int image_h,image_w;
    unsigned int convert_lie;
    byte select[2]={0,255};
	for(image_h=0;image_h<OV7725_EAGLE_H;image_h++)
	{	
		for(image_w=0;image_w<OV7725_EAGLE_W;image_w=image_w+8)
	    {
			 convert_lie=image_w>>3;
			 Get_extract_image[image_h][image_w]=select[(Dest_data[image_h][convert_lie]>>7)&0x01];
			 Get_extract_image[image_h][image_w+1]=select[(Dest_data[image_h][convert_lie]>>6)&0x01];
			 Get_extract_image[image_h][image_w+2]=select[(Dest_data[image_h][convert_lie]>>5)&0x01];
			 Get_extract_image[image_h][image_w+3]=select[(Dest_data[image_h][convert_lie]>>4)&0x01];
			 Get_extract_image[image_h][image_w+4]=select[(Dest_data[image_h][convert_lie]>>3)&0x01];
			 Get_extract_image[image_h][image_w+5]=select[(Dest_data[image_h][convert_lie]>>2)&0x01];
			 Get_extract_image[image_h][image_w+6]=select[(Dest_data[image_h][convert_lie]>>1)&0x01];
			 Get_extract_image[image_h][image_w+7]=select[(Dest_data[image_h][convert_lie])&0x01];
	    }
	}
}

/************************************************** 
** 函数名称:                                                                                                                                                
** 功能描述: 数值计算
** 输    入: 运算量                            
** 输    出: 运算结果                             
** 说明：                                                   
***************************************************/
//取两数之差的绝对值
int absolute(int x1,int x2)
{		 
  if(x1>x2)
  {
    return(x1-x2);
  }
  else
  {
    return(x2-x1);		  
  }
}
//取最大值
byte  min(byte x1,byte  x2)
{
  if(x1>x2)
  {  
    return(x2);
  }
  else
  { 
    return(x1);
  }
}

/*
** 函数名称: BlackCount
** 功能描述:                                                                                 
** 输    入:                                                                       
** 输    出: row指向行值，comlom指向列
** 说明    : 
***************************************************/
byte BlackCount(byte row ,byte comlom)
{
	byte count=0;
	byte i;
	byte j;
	if(row<2)
		row=1;
	else
		if(row>NEED_ROW-3)
			row=NEED_ROW-2;
	
	
	if(comlom<2)
		comlom=2;
	else
		if(comlom>OV7725_EAGLE_W-2)
			comlom=OV7725_EAGLE_W-2;
	
	
	    i=row;
		for(j=comlom-1;j<=comlom+1;j++)
		{
			if(Get_extract_image[i][j]==0xff)
				count++;	
		}
	return count;
}
/************************************************** 
** 函数名称: 		FarCount
** 功能描述:      left_line[i]  右射线：（70,50)
               right_line[i]    左射线：(10,30）                                                                    
** 输    入:         R_Count：左端射线与左黑线的交点                                                              
** 输    出:         L_Count：右端射线与右黑线的交点
** 说明    :      Mid_farCount  远端有效视野
***************************************************/
void FarCount(void)
{
	   byte i,c;
	   byte max_c,min_c;
	  // byte flag_road=0x00;
	   byte Mid_Count=0;
	   Mid_farCount=NEED_ROW-1;
	   L_Count=NEED_ROW-1;
	   RR_Count=NEED_ROW-1;
	   
	   if(My_Road_Condition [NEED_ROW-1].centre_position-15>My_Road_Condition [NEED_ROW-1].L_position) min_c=My_Road_Condition [NEED_ROW-1].centre_position-15;
	   else min_c=My_Road_Condition [NEED_ROW-1].L_position+3;
	   if(My_Road_Condition [NEED_ROW-1].centre_position+15<My_Road_Condition [NEED_ROW-1].R_position) max_c=My_Road_Condition [NEED_ROW-1].centre_position+15;
	   else max_c=My_Road_Condition [NEED_ROW-1].R_position-3;
	   
	   for(c=min_c;c<max_c;c=c+3)
	   {
		  for(i=NEED_ROW-1;i>1;i--) //中心线：列=40；
	      {   
	       if(BlackCount(i,c)>=Black_Limit)
	       {
	          Mid_Count=i;//远端有效视野 Mid_farCount
	          break;
	       }
	        else 
	        Mid_Count=1;  
	      }
		  if(Mid_Count<Mid_farCount)  
			  {Mid_farCount=Mid_Count;mid_value=c;}
	   }
	   for(i=NEED_ROW-1;i>1;i--)    //右射线：   
	     {   
	                 //最大34+32*17/34=50
	       if(BlackCount(i,set_left[i])>=Black_Limit)          //  L_Count：右端射线与右黑线的交点
	       {
	          L_Count=i;
	          break;
	       } 
	       else
	    	  L_Count=1;
	     }
	   for(i=NEED_ROW-1;i>1;i--)  //左射线：
	     {   
		  
		   if(BlackCount(i,set_right[i])>=Black_Limit)             //   R_Count：左端射线与左黑线的交点
	       {
	          RR_Count=i;
	          break;
	       }
		   else
			   RR_Count=1;
	     }
}




byte edge_image[60][80];
void get_edge_image(void)
{
	byte lie,hang;
	for(hang=0;hang<60;hang++)
	{	for(lie=0;lie<80;lie++)
		{
			edge_image[hang][lie]=255;
		}
	}
	for(hang=0;hang<60;hang++)
	{  
		lie=My_Road_Condition[hang].L_position;
		edge_image[hang][lie]=128;
		lie=My_Road_Condition[hang].R_position;
		edge_image[hang][lie]=128;
		lie=carry_centre[59-hang];
		edge_image[hang][lie]=128;
	}	
	for(hang=valid_hang;hang<NEED_ROW;hang++)
	{
		lie=My_Road_Condition[hang].L_position;
		edge_image[hang][lie]=0;
		lie=My_Road_Condition[hang].R_position;
		edge_image[hang][lie]=0;
		lie=carry_centre[59-hang];
		edge_image[hang][lie]=0;
	}
}


void get_first_double_edge(byte line_need)
{
		 byte LL_line,LR_line;
		 byte RL_line,RR_line;
		 byte j;
		 if((last_left_pos-COLUMU_diff_first)>left_lie)
		 {	 
			 LL_line=last_left_pos-COLUMU_diff_first;
			 LR_line=last_left_pos+COLUMU_diff_first;
		 }
		 else
		 {
			 LL_line=left_lie;
			 LR_line=last_left_pos+COLUMU_diff_first;
		 }
		 
		 if((last_right_pos+COLUMU_diff_first)<COLUMN-1)
		 {
			 RL_line=last_right_pos-COLUMU_diff_first;
			 RR_line=last_right_pos+COLUMU_diff_first;
		 }
		 else
		 {
			 RL_line=last_right_pos-COLUMU_diff_first;
			 RR_line=COLUMN-1;
		 } 
		 /***********************************************检测左线**********************************************/
                   for(j=LR_line;j>LL_line;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
				   {
					 if(j<left_lie+1) j=left_lie+1;
					 if(j>COLUMN-8) j=COLUMN-8;
					 if(Get_extract_image[line_need][j]==0xff&&Get_extract_image[line_need][j+1]==0x00
							 &&Get_extract_image[line_need][j+2]==0x00&&Get_extract_image[line_need][j+3]==0x00
							 &&Get_extract_image[line_need][j+4]==0x00&&Get_extract_image[line_need][j+5]==0x00
							 &&Get_extract_image[line_need][j+6]==0x00&&Get_extract_image[line_need][j+7]==0x00
							 &&Get_extract_image[line_need][j+8]==0x00)
					 {   
						 My_Road_Condition[line_need].road_Left_flag=TRUE;
						 My_Road_Condition[line_need].L_position=j;
						 My_Road_Condition[line_need].road_typex=mid_LINE;
						 break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_Left_flag=FALSE;  
						 My_Road_Condition[line_need].road_typex=no_LINE;
					 }
				    }
      			 if(My_Road_Condition[line_need].road_Left_flag==FALSE)			//上次右线边线有效
      			 {
					 for(j=last_right_pos-15;j>left_lie;j--)  								//基于中心找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
					 {
						 //if(BlackCount(line_need,j)>Black_Limit)   //从中间往左边扫
						 if(j<left_lie+1) j=left_lie;
						 if(j>COLUMN-9) j=COLUMN-9;
						 if(Get_extract_image[line_need][j]==0xff&&Get_extract_image[line_need][j+1]==0x00
								 &&Get_extract_image[line_need][j+2]==0x00&&Get_extract_image[line_need][j+3]==0x00
								 &&Get_extract_image[line_need][j+4]==0x00&&Get_extract_image[line_need][j+5]==0x00
								 &&Get_extract_image[line_need][j+6]==0x00&&Get_extract_image[line_need][j+7]==0x00
								 &&Get_extract_image[line_need][j+8]==0x00)
						 {   
							 My_Road_Condition[line_need].road_Left_flag=TRUE;
							 My_Road_Condition[line_need].L_position=j;
							 My_Road_Condition[line_need].road_typex=mid_LINE;
							 break;
						 }
						 else
						 {
							 My_Road_Condition[line_need].road_Left_flag=TRUE; 
							 My_Road_Condition[line_need].L_position=left_lie;
							 My_Road_Condition[line_need].road_typex=no_LINE;
						 }
					 }
				 }
      			last_left_pos=My_Road_Condition[line_need].L_position;
      /***********************************************检测右线**********************************************/
				 for(j=RL_line;j<RR_line;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
				 {
					// if(BlackCount(line_need,j)>Black_Limit)
					 if(j<left_lie+8) j=left_lie+8;
					 if(j>COLUMN-1) j=COLUMN-1;
					 if(Get_extract_image[line_need][j-8]==0x00&&Get_extract_image[line_need][j-7]==0x00
							  &&Get_extract_image[line_need][j-6]==0x00&&Get_extract_image[line_need][j-5]==0x00
	                          &&Get_extract_image[line_need][j-4]==0x00&&Get_extract_image[line_need][j-3]==0x00
	                          &&Get_extract_image[line_need][j-2]==0x00&&Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0xff)
					 {
					 	My_Road_Condition[line_need].R_position=j;
					 	My_Road_Condition[line_need].road_right_flag=TRUE;
					 	My_Road_Condition[line_need].road_typex=mid_LINE;
					 	break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_right_flag=FALSE;
					 	 My_Road_Condition [line_need].road_typex=no_LINE;
					 } 
				 }
				 if(My_Road_Condition[line_need].road_right_flag==FALSE)
				 {
					 for(j=last_left_pos+15;j<COLUMN;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
					 {
						// if(BlackCount(line_need,j)>Black_Limit)
						 if(j<left_lie+8) j=left_lie+8;
						 if(j>COLUMN) j=COLUMN-1;
						 if(Get_extract_image[line_need][j-8]==0x00&&Get_extract_image[line_need][j-7]==0x00
								  &&Get_extract_image[line_need][j-6]==0x00&&Get_extract_image[line_need][j-5]==0x00
		                          &&Get_extract_image[line_need][j-4]==0x00&&Get_extract_image[line_need][j-3]==0x00
		                          &&Get_extract_image[line_need][j-2]==0x00&&Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0xff)
						 {
						 	My_Road_Condition[line_need].R_position=j;
						 	My_Road_Condition[line_need].road_right_flag=TRUE;
						 	My_Road_Condition[line_need].road_typex=mid_LINE;
						 	break;
						 }
						 else
						 {
							 My_Road_Condition[line_need].road_right_flag=TRUE;
							 My_Road_Condition[line_need].R_position=COLUMN-1;
						 	 My_Road_Condition [line_need].road_typex=no_LINE;
						 } 
					 }
				 }
		   last_right_pos=My_Road_Condition[line_need].R_position;
}
 
/************************************************** 
 ** 函数名称: GetDoubleEage
 ** 功能描述: 提取两边黑线坐标                                                                                  
 ** 说明    : 对图像进行处理，得到有效行数、近端无效行数，对十字交叉道进行牵引铺路
 ***************************************************/

 void  Get_left_back_Eage(byte line_need)
 {
	 byte LL_line,LR_line;
	 byte j;
	 byte center;
	 if(L_Count<RR_Count&&L_Count<Mid_farCount)     center=set_left[line_need];
	 else if(RR_Count<L_Count&&RR_Count<Mid_farCount) center=set_right[line_need];
	 else center=mid_value;
	 if((My_Road_Condition[line_need+1].L_position-COLUMU_diff)>left_lie)
	 {	 
		 LL_line=My_Road_Condition[line_need+1].L_position-COLUMU_diff;
		 LR_line=My_Road_Condition[line_need+1].L_position+COLUMU_diff;
	 }
	 else
	 {
		 LL_line=left_lie;
		 LR_line=My_Road_Condition[line_need+1].L_position+COLUMU_diff;
	 }
	 
      /*************************************扫描左边沿**********************************/	  
		    for(j=LR_line;j>LL_line;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
			   {
				 if(j<left_lie+1) j=left_lie+1;
				 if(j>=COLUMN-2) j=COLUMN-2;
				 if(Get_extract_image[line_need][j-1]==0xff&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0x00)
				 {   
					 My_Road_Condition[line_need].road_Left_flag=TRUE;
					 My_Road_Condition[line_need].L_position=j;
					 My_Road_Condition[line_need].road_typex=mid_LINE;
					 break;
				 }
				 else
				 {
					 My_Road_Condition[line_need].road_Left_flag=FALSE;  
					 My_Road_Condition[line_need].road_typex=no_LINE;
				 }
			    if(Get_extract_image[line_need][LR_line]==0xff)   break;
			   }
			 if(My_Road_Condition[line_need].road_Left_flag==FALSE)
			 {   
				if(LL_line==left_lie) 
					{
					for(j=center;j>left_lie;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
				    {
					 //if(BlackCount(line_need,j)>Black_Limit)   //从中间往左边扫
					 if(j<left_lie+1) j=left_lie+1;
					 if(j>=COLUMN-2) j=COLUMN-2;
					 if(Get_extract_image[line_need][j-1]==0xff&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0x00)
					 {   
						 My_Road_Condition[line_need].road_Left_flag=TRUE;
						 My_Road_Condition[line_need].L_position=j;
						 My_Road_Condition[line_need].road_typex=mid_LINE;
						 break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_Left_flag=TRUE; 
						 My_Road_Condition[line_need].L_position=left_lie;
						 My_Road_Condition[line_need].road_typex=no_LINE;
					 }
				    }
					}
				else
				 {
					for(j= center;j>left_lie;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
				    {
					 //if(BlackCount(line_need,j)>Black_Limit)   //从中间往左边扫
					 if(j<left_lie+1) j=left_lie+1;
					 if(j>=COLUMN-2) j=COLUMN-2;
					 if(Get_extract_image[line_need][j-1]==0xff&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0x00)
					 {   
						 My_Road_Condition[line_need].road_Left_flag=TRUE;
						 My_Road_Condition[line_need].L_position=j;
						 My_Road_Condition[line_need].road_typex=mid_LINE;
						 break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_Left_flag=TRUE; 
						 My_Road_Condition[line_need].L_position=left_lie;
						 My_Road_Condition[line_need].road_typex=no_LINE;
					 }
				    }
				 }
			   }
			    if(line_need>=5&&line_need<=50)
			   {   
			    	if(My_Road_Condition[line_need+2].L_position>My_Road_Condition[line_need].L_position&&
						  My_Road_Condition[line_need+2].L_position>=My_Road_Condition[line_need+4].L_position
						  &&left_guaidian1==0
						 &&(My_Road_Condition[line_need+2].L_position*2-My_Road_Condition[line_need+4].L_position-My_Road_Condition[line_need].L_position)>=5)
						  {left_guaidian1=1;left_guaidian_flag[0]=line_need+2;}
	           }
			/*    if(line_need>30&&line_need<55)
			    {
			    	if(My_Road_Condition[line_need+2].L_position>My_Road_Condition[line_need].L_position&&
			    		My_Road_Condition[line_need+2].L_position>=My_Road_Condition[line_need+4].L_position
			    		&&left_guaidian1==0
			    		&&(My_Road_Condition[line_need+2].L_position*2-My_Road_Condition[line_need+4].L_position-My_Road_Condition[line_need].L_position)>=2)
			    		{left_guaidian1=1;left_guaidian_flag[0]=line_need+2;}
			    }*/
			    
			    if(line_need>valid_hang+3&&line_need<55)
			    {
			    	if(My_Road_Condition[line_need+3].L_position>My_Road_Condition[line_need].L_position&&
			    		My_Road_Condition[line_need+3].L_position>=My_Road_Condition[line_need+6].L_position
			    		&&(My_Road_Condition[line_need+3].L_position*2-My_Road_Condition[line_need+6].L_position-My_Road_Condition[line_need].L_position)>=5
			    		&&My_Road_Condition[line_need].L_position==left_lie)
			    	{ren_left_flag=1;ren_hang=line_need+3;}
			    }
			    
}

 void Get_right_back_Eage(byte line_need)
 {
	 byte RL_line,RR_line;
	 byte j;
	 byte center;
	 if(L_Count<RR_Count&&L_Count<Mid_farCount)     center=set_left[line_need];
	 else if(RR_Count<L_Count&&RR_Count<Mid_farCount) center=set_right[line_need];
	 else center=mid_value;
	 if((My_Road_Condition[line_need+1].R_position+COLUMU_diff)<COLUMN-1)
		 {
			 RL_line=My_Road_Condition[line_need+1].R_position-COLUMU_diff;
			 RR_line=My_Road_Condition[line_need+1].R_position+COLUMU_diff;
		 }
		 else
		 {
			 RL_line=My_Road_Condition[line_need+1].R_position-COLUMU_diff;
			 RR_line=COLUMN-1;
		 }
		/***********************************扫描右边沿******************************************/
                 if(RR_line>COLUMN-1)   RR_line=COLUMN-1;
 		           for(j=RL_line;j<RR_line;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
				 {
					// if(BlackCount(line_need,j)>Black_Limit)
					 if(j<left_lie+1) j=left_lie+1;
					 if(j>=COLUMN-2) j=COLUMN-2;
					 if(Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0xff)
					 {
					 	My_Road_Condition[line_need].R_position=j;
					 	My_Road_Condition[line_need].road_right_flag=TRUE;
					 	My_Road_Condition[line_need].road_typex=mid_LINE;
					 	break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_right_flag=FALSE;
					 	 My_Road_Condition [line_need].road_typex=no_LINE;
					 } 
					 if(Get_extract_image[line_need][RL_line]==0xff)   break;
				 }
           
				 if(My_Road_Condition[line_need].road_right_flag==FALSE)
				 {
					 if(RR_line==(COLUMN-1))  
						 {
						   for(j=center;j<COLUMN-1;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
						 	{
						 		// if(BlackCount(line_need,j)>Black_Limit)
						 		if(j<left_lie+1) j=left_lie+1;
						 		if(j>=COLUMN-2) j=COLUMN-2;
						 		if(Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0xff)
						 		{
						 			My_Road_Condition[line_need].R_position=j;
						 			My_Road_Condition[line_need].road_right_flag=TRUE;
						 			My_Road_Condition[line_need].road_typex=mid_LINE;
						 			break;
						 		}
						 		else
						 		{
						 			 My_Road_Condition[line_need].road_right_flag=TRUE;
						 			My_Road_Condition[line_need].R_position=COLUMN-1;
						 			My_Road_Condition [line_need].road_typex=no_LINE;
						 		} 
						 	}
						 }
					 else
					 {
					    for(j=center;j<COLUMN-1;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
					   {
						// if(BlackCount(line_need,j)>Black_Limit)
						 if(j<left_lie+1) j=left_lie+1;
						 if(j>=COLUMN-2) j=COLUMN-2;
						 if(Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0xff)
						 {
						 	My_Road_Condition[line_need].R_position=j;
						 	My_Road_Condition[line_need].road_right_flag=TRUE;
						 	My_Road_Condition[line_need].road_typex=mid_LINE;
						 	break;
						 }
						 else
						 {
							 My_Road_Condition[line_need].road_right_flag=TRUE;
							 My_Road_Condition[line_need].R_position=COLUMN-1;
						 	 My_Road_Condition [line_need].road_typex=no_LINE;
						 } 
					   }
					 }
				 }
				  if(line_need>=5&&line_need<=50)
				   {   
					  if(My_Road_Condition[line_need+2].R_position<My_Road_Condition[line_need].R_position&&
							  My_Road_Condition[line_need+2].R_position<=My_Road_Condition[line_need+4].R_position
							  &&right_guaidian1==0
							 &&My_Road_Condition[line_need+4].R_position+My_Road_Condition[line_need].R_position-My_Road_Condition[line_need+2].R_position*2>=5)
							  {right_guaidian1=1;right_guaidian_flag[0]=line_need+2;}
		           }
			/*	  if(line_need>30&&line_need<55)
				  {
					 if(My_Road_Condition[line_need+2].R_position<My_Road_Condition[line_need].R_position&&
					  	 My_Road_Condition[line_need+2].R_position<=My_Road_Condition[line_need+4].R_position
					  	&&right_guaidian1==0
					  	&&My_Road_Condition[line_need+4].R_position+My_Road_Condition[line_need].R_position-My_Road_Condition[line_need+2].R_position*2>=2)
					  	{right_guaidian1=1;right_guaidian_flag[0]=line_need+2;}
				  }*/
				  if(line_need>valid_hang+3&&line_need<55)
				  {
					  if(My_Road_Condition[line_need+3].R_position<My_Road_Condition[line_need].R_position&&
					  		 My_Road_Condition[line_need+3].R_position<=My_Road_Condition[line_need+6].R_position
					  		 &&My_Road_Condition[line_need+6].R_position+My_Road_Condition[line_need].R_position-My_Road_Condition[line_need+4].R_position*2>=5
					  		&&My_Road_Condition[line_need].R_position==COLUMN-1)
						  {ren_right_flag=1;ren_hang=line_need+2;}
				  }
 }


 void Get_left_cross_Eage(byte line_need)
 {
	 byte LL_line,LR_line;
	 byte j;
	 byte center;
	 if(L_Count<RR_Count&&L_Count<Mid_farCount)     center=set_left[line_need];
	 else if(RR_Count<L_Count&&RR_Count<Mid_farCount) center=set_right[line_need];
	 else center=mid_value;
	 float left_flag;byte left_black_num=0;
	 left_flag=(float)(My_Road_Condition[left_guaidian_flag[0]].L_position-My_Road_Condition[OV7725_EAGLE_H-1].L_position)
			     *(float)(OV7725_EAGLE_H-line_need)/(float)(OV7725_EAGLE_H-left_guaidian_flag[0])+My_Road_Condition[OV7725_EAGLE_H-1].L_position;
	 if(((byte)left_flag-5)>left_lie)
	 {	 
		 LL_line=(byte)left_flag-5;
		 LR_line=(byte)left_flag+5;
	 }
	 else
	 {
		 LL_line=left_lie;
		 LR_line=(byte)left_flag+5;
	 }
	 
      /*************************************扫描左边沿**********************************/
			  for(j=LR_line;j>LL_line;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
			   {
				 if(j<left_lie+1) j=left_lie+1;
				 if(j>COLUMN-2) j=COLUMN-2;
				 if(Get_extract_image[line_need][j-1]==0xff&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0x00)
				 {   
					 My_Road_Condition[line_need].road_Left_flag=TRUE;
					 My_Road_Condition[line_need].L_position=j-1;
					 My_Road_Condition[line_need].road_typex=mid_LINE;
					 left_guaidian2=1;
					 if(line_need>=15)
					 left_guaidian_flag[1]=line_need-5;
					 else left_guaidian_flag[1]=line_need;
					 break;
				 }
				 else
				 {
					 My_Road_Condition[line_need].road_Left_flag=FALSE;  
					 My_Road_Condition[line_need].road_typex=no_LINE;
				 }
				 if(Get_extract_image[line_need][j]==0xff)
				 {
					 left_black_num++;
					 if(left_black_num>=8)    {left_guaidian2=1;left_guaidian_flag[1]=line_need;} 
				 }
			    }			 
			 if(My_Road_Condition[line_need].road_Left_flag==FALSE)
			 {  
				// My_Road_Condition[line_need].L_position=(byte)left_flag;
			  //  Get_extract_image[line_need][My_Road_Condition[line_need].L_position]=255;
					for(j= center;j>left_lie;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
				    {
					 //if(BlackCount(line_need,j)>Black_Limit)   //从中间往左边扫
					 if(j<left_lie+1) j=left_lie+1;
					 if(j>=COLUMN-2) j=COLUMN-2;
					 if(Get_extract_image[line_need][j-1]==0xff&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0x00)
					 {   
						 My_Road_Condition[line_need].road_Left_flag=TRUE;
						 My_Road_Condition[line_need].L_position=j;
						 My_Road_Condition[line_need].road_typex=mid_LINE;
						 break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_Left_flag=TRUE; 
						 My_Road_Condition[line_need].L_position=left_lie;
						 My_Road_Condition[line_need].road_typex=no_LINE;
					 }
				    }
			 } 
}

void Get_right_cross_Eage(byte line_need)
{
	 byte RL_line,RR_line;
	 byte j;
	 byte center;
	 if(L_Count<RR_Count&&L_Count<Mid_farCount)     center=set_left[line_need];
	 else if(RR_Count<L_Count&&RR_Count<Mid_farCount) center=set_right[line_need];
	 else center=mid_value;
	 float right_flag;
	 right_flag=My_Road_Condition[OV7725_EAGLE_H-1].R_position-(float)(My_Road_Condition[OV7725_EAGLE_H-1].R_position-My_Road_Condition[right_guaidian_flag[0]].R_position)
					     *(float)(OV7725_EAGLE_H-line_need)/(float)(OV7725_EAGLE_H-right_guaidian_flag[0]);
	 if((right_flag+COLUMU_diff)<COLUMN-1)
	 {
		 RL_line=right_flag-COLUMU_diff;
		 RR_line=right_flag+COLUMU_diff;
	 }
	 else
	 {
		 RL_line=right_flag-COLUMU_diff;
		 RR_line=COLUMN-1;
	 }
		/***********************************扫描右边沿******************************************/	 
				 for(j=RL_line;j<RR_line;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
				 {
					// if(BlackCount(line_need,j)>Black_Limit)
					 if(j<left_lie+1) j=left_lie+1;
					 if(j>COLUMN-2) j=COLUMN-2;
					 if(Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0xff)
					 {
					 	My_Road_Condition[line_need].R_position=j+1;
					 	My_Road_Condition[line_need].road_right_flag=TRUE;
					 	My_Road_Condition[line_need].road_typex=mid_LINE;
						right_guaidian2=1;
						if(line_need>=15)
						right_guaidian_flag[1]=line_need-5;
						else right_guaidian_flag[1]=line_need;
					 	break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_right_flag=FALSE;
					 	 My_Road_Condition [line_need].road_typex=no_LINE;
					 } 
				 }
				 if(My_Road_Condition[line_need].road_right_flag==FALSE)
				 {
					// My_Road_Condition[line_need].R_position=(byte)right_flag;
					//  Get_extract_image[line_need][My_Road_Condition[line_need].R_position]=255;
					    for(j=center;j<COLUMN-1;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
					   {
						// if(BlackCount(line_need,j)>Black_Limit)
						 if(j<left_lie+1) j=left_lie+1;
						 if(j>=COLUMN-2) j=COLUMN-2;
						 if(Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0xff)
						 {
						 	My_Road_Condition[line_need].R_position=j;
						 	My_Road_Condition[line_need].road_right_flag=TRUE;
						 	My_Road_Condition[line_need].road_typex=mid_LINE;
						 	break;
						 }
						 else
						 {
							 My_Road_Condition[line_need].road_right_flag=TRUE;
							 My_Road_Condition[line_need].R_position=COLUMN-1;
						 	 My_Road_Condition [line_need].road_typex=no_LINE;
						 } 
					   }
				 }	
}
 

void get_second_centre(byte *valid)
{
	 byte j;
	 byte lie_tiao1=0,lie_tiao2=0;//纵向扫线判断
	 signed char my_row;
	 zhangai_left=0;zhangai_right=0;
	 roadtype=normal_road;
	 left_guaidian1=0;left_guaidian2=0;
	 right_guaidian1=0;right_guaidian2=0;
	 ren_left_flag=0;ren_right_flag=0;
	 byte diuxian_left=0,diuxian_right=0;
	 byte line_need;
	 danbian_flag=0;
	 byte diuxian_row=0;byte diuxian_dir=0;byte my_hang,my_lie;byte lie_top[80];//回拐参数
	 byte diuxian_flag=0;
	 /***************************************************************第一行检测**************************************************/
	 line_need=NEED_ROW-1;
	 if(start_flag)
	 {	 start_flag=FALSE;
	 			//Get_Double_Eage(my_row);
				 for(j=COLUMN-9;j>left_lie;j--)  								//基于上一行找左黑线     开始于做显得右边，结束于左线有左线，向左边扫描
				 {
					 if(j<left_lie+1) j=left_lie+1;
					 if(j==COLUMN) j=COLUMN-1;
					 if(Get_extract_image[line_need][j-1]==0xff&&Get_extract_image[line_need][j]==0x00&&Get_extract_image[line_need][j+1]==0x00
							 &&Get_extract_image[line_need][j+2]==0x00&&Get_extract_image[line_need][j+3]==0x00
							 &&Get_extract_image[line_need][j+4]==0x00&&Get_extract_image[line_need][j+5]==0x00
							 &&Get_extract_image[line_need][j+6]==0x00&&Get_extract_image[line_need][j+7]==0x00)
					 {   
						 My_Road_Condition[line_need].road_Left_flag=TRUE;
						 My_Road_Condition[line_need].L_position=j-1;
						 My_Road_Condition[line_need].road_typex=mid_LINE;
						 if(j-1>(COLUMN+left_lie)/2)  
						 {
							 My_Road_Condition[line_need].L_position=left_lie;
							 My_Road_Condition[line_need].road_typex=mid_LINE;
						 }
						 break;
					 }
					 else
					 {
						 My_Road_Condition[line_need].road_Left_flag=TRUE;
						 My_Road_Condition[line_need].L_position=left_lie;
						 My_Road_Condition[line_need].road_typex=no_LINE;
					 }
				 }
				 last_left_pos=My_Road_Condition[line_need].L_position;
				for(j=left_lie+1;j<COLUMN-9;j++)							//开始于右线的左边，结束于右线的右边，向右边扫描
				 {
						// if(BlackCount(line_need,j)>Black_Limit)
						 if(j<left_lie+1) j=left_lie+1;
						 if(j==COLUMN) j=COLUMN-1;
						 if(Get_extract_image[line_need][j-1]==0x00&&Get_extract_image[line_need][j]==0x00
								  &&Get_extract_image[line_need][j+1]==0x00&&Get_extract_image[line_need][j+2]==0x00
		                          &&Get_extract_image[line_need][j+3]==0x00&&Get_extract_image[line_need][j+4]==0x00
		                          &&Get_extract_image[line_need][j+5]==0x00&&Get_extract_image[line_need][j+6]==0x00&&Get_extract_image[line_need][j+7]==0xff)
						 {
						 	My_Road_Condition[line_need].R_position=j+7;
						 	My_Road_Condition[line_need].road_right_flag=TRUE;
						 	My_Road_Condition[line_need].road_typex=mid_LINE;
							 if(j-1<(COLUMN+left_lie)/2)  
							 {
								 My_Road_Condition[line_need].R_position=COLUMN-1;
								 My_Road_Condition[line_need].road_typex=mid_LINE;
							 }
						 	break;
						 }
						 else
						 {
							 My_Road_Condition[line_need].road_right_flag=TRUE;
							 My_Road_Condition[line_need].R_position=COLUMN-1;
						 	 My_Road_Condition [line_need].road_typex=no_LINE;
						 } 
			    }
				last_right_pos=My_Road_Condition[line_need].R_position;
	 }
	 else                                                                //如果start_flag为false
	 {  
	    get_first_double_edge(line_need);
	 }
     if(My_Road_Condition[line_need].R_position>COLUMN-1)  My_Road_Condition[line_need].R_position=COLUMN-1;
     if(My_Road_Condition[line_need].L_position<left_lie)  My_Road_Condition[line_need].L_position=left_lie;
	 My_Road_Condition[line_need].centre_position=(My_Road_Condition[line_need].R_position+My_Road_Condition[line_need].L_position)>>1;	 							
	 My_Road_Condition[line_need].road_condition=TRUE;
	 /*						 						
	 carry_row[my_row_num]=my_row_num;
	 carry_centre[my_row_num]=My_Road_Condition[line_need].centre_position;
	 my_row_num++;*/
	 FarCount();
	 j=min(RR_Count,L_Count);
	 valid_hang=min(j,Mid_farCount);
/*****************************************************检测剩余行**************************************************************/
	 for(my_row=NEED_ROW-2;my_row>valid_hang;my_row--)
	 {  
		if(left_guaidian1==0) Get_left_back_Eage(my_row);
		else Get_left_cross_Eage(my_row);
		if(right_guaidian1==0) Get_right_back_Eage(my_row);
		else Get_right_cross_Eage(my_row);
	 	/*
	 	if((My_Road_Condition[my_row].L_position==0&&My_Road_Condition[my_row].R_position<35)
	 			||(My_Road_Condition[my_row].L_position>COLUMN-35&&My_Road_Condition[my_row].R_position>=COLUMN-1))
	 		{valid_hang=my_row;break;}
	 	if(My_Road_Condition[my_row].L_position+8>My_Road_Condition[my_row].R_position)
	 	{valid_hang=my_row;break;}
	 	if(My_Road_Condition[my_row+1].L_position-My_Road_Condition[my_row].L_position>15&&my_row<20) break;
	 	if(My_Road_Condition[my_row].R_position-My_Road_Condition[my_row+1].R_position>15&&my_row<20) break;*/
		
	   /****************************************十字拐点上边沿检测***********************************************/
	/*	 float left_flag,right_flag;
		 if(left_guaidian1&&!left_guaidian2)
		  {
			 left_flag=(float)(My_Road_Condition[left_guaidian_flag[0]].L_position-My_Road_Condition[OV7725_EAGLE_H-1].L_position)
		 			     *(float)(OV7725_EAGLE_H-my_row)/(float)(OV7725_EAGLE_H-left_guaidian_flag[0])+My_Road_Condition[OV7725_EAGLE_H-1].L_position;
		//	 Get_extract_image[my_row][(byte)left_flag]=100;
			 if(My_Road_Condition[my_row].L_position<left_flag+15&&My_Road_Condition[my_row].L_position>left_flag-1)
				 {left_guaidian2=1;left_guaidian_flag[1]=my_row;}		    
		  }
		 if(right_guaidian1&&!right_guaidian2)	
		 {
			 right_flag=My_Road_Condition[OV7725_EAGLE_H-1].R_position-(float)(My_Road_Condition[OV7725_EAGLE_H-1].R_position-My_Road_Condition[right_guaidian_flag[0]].R_position)
			 					     *(float)(OV7725_EAGLE_H-my_row)/(float)(OV7725_EAGLE_H-right_guaidian_flag[0]);
			 if(My_Road_Condition[my_row].R_position<right_flag+1&&My_Road_Condition[my_row].R_position>right_flag-15)
			 	{right_guaidian2=1;right_guaidian_flag[1]=my_row;}
			 Get_extract_image[my_row][(byte)right_flag]=100;
		 }*/
		 /********************************************障碍检测**************************************************************/
		if(my_row<=50) 
		{  int test_num;
		   test_num=(My_Road_Condition[my_row+1].L_position+My_Road_Condition[my_row+2].L_position+My_Road_Condition[my_row+3].L_position+My_Road_Condition[my_row].L_position)/4;
		   test_num=abs(My_Road_Condition[my_row].L_position-test_num)
				    +abs(My_Road_Condition[my_row+1].L_position-test_num)
				    +abs(My_Road_Condition[my_row+2].L_position-test_num)
				    +abs(My_Road_Condition[my_row+3].L_position-test_num);
			if(My_Road_Condition[my_row+4].L_position-My_Road_Condition[my_row+6].L_position>=8
			&&My_Road_Condition[my_row+4].L_position-My_Road_Condition[my_row+7].L_position>=8
			&&My_Road_Condition[my_row+4].L_position-My_Road_Condition[my_row+8].L_position>=8
			&&My_Road_Condition[my_row+4].L_position-My_Road_Condition[my_row+9].L_position>=8
			&&My_Road_Condition[my_row+6].L_position!=left_lie
			&&My_Road_Condition[my_row+7].L_position!=left_lie
			&&My_Road_Condition[my_row+8].L_position!=left_lie
			&&My_Road_Condition[my_row+9].L_position!=left_lie
			&&(test_num<=6)
		    &&my_row>15)   
		 {zhangai_left=1;}
		test_num=(My_Road_Condition[my_row+1].R_position+My_Road_Condition[my_row+2].R_position+My_Road_Condition[my_row+3].R_position+My_Road_Condition[my_row].R_position)/4;
		test_num=abs(My_Road_Condition[my_row].R_position-test_num)
					    +abs(My_Road_Condition[my_row+1].R_position-test_num)
					    +abs(My_Road_Condition[my_row+2].R_position-test_num)
					    +abs(My_Road_Condition[my_row+3].R_position-test_num);
		 if(My_Road_Condition[my_row+6].R_position-My_Road_Condition[my_row+4].R_position>=8
			&&My_Road_Condition[my_row+7].R_position-My_Road_Condition[my_row+4].R_position>=8
		    &&My_Road_Condition[my_row+8].R_position-My_Road_Condition[my_row+4].R_position>=8
		    &&My_Road_Condition[my_row+9].R_position-My_Road_Condition[my_row+4].R_position>=8
		    &&My_Road_Condition[my_row+6].R_position!=COLUMN-1
		    &&My_Road_Condition[my_row+7].R_position!=COLUMN-1
		    &&My_Road_Condition[my_row+8].R_position!=COLUMN-1
		    &&My_Road_Condition[my_row+9].R_position!=COLUMN-1
		    &&(test_num<=6)
			&&my_row>15)  
		 {zhangai_right=1;}
		}
		/*******************************************单边补线**************************************************************/
		 
		// if(My_Road_Condition[my_row].R_position==COLUMN-1&&My_Road_Condition[NEED_ROW-1].R_position!=COLUMN-1&&diuxian_flag==0)
		 if(My_Road_Condition[my_row].R_position==COLUMN-1&&My_Road_Condition[my_row+1].R_position!=COLUMN-1&&diuxian_flag==0&&my_row<50)
			 {
			  diuxian_row=my_row+1;
			  diuxian_dir=2;
			 }
		// if(My_Road_Condition[my_row].L_position==0&&My_Road_Condition[NEED_ROW-1].L_position!=0&&diuxian_flag==0)
		 if(My_Road_Condition[my_row].L_position==left_lie&&My_Road_Condition[my_row+1].L_position!=left_lie&&diuxian_flag==0&&my_row<50)
		     {diuxian_row=my_row+1;diuxian_dir=1;}
		 if(diuxian_dir!=0&&diuxian_flag==0)
		 {
		   lie_tiao1=0;lie_tiao2=0;
		   for(my_lie=My_Road_Condition[diuxian_row].L_position;my_lie<=My_Road_Condition[diuxian_row].R_position;my_lie++)
		   {
			  for(my_hang=diuxian_row;my_hang>0;my_hang--)
			  {	 
				  lie_top[my_lie]=0;
				  if(Get_extract_image[my_hang][my_lie]==255)
			        {
					  //Get_extract_image[my_hang][my_lie]=175;
			          lie_top[my_lie]=my_hang;break;
			        }
			  }
		      if(my_lie>My_Road_Condition[diuxian_row].L_position+3&&my_lie<My_Road_Condition[diuxian_row].R_position-1&&my_lie>=left_lie+17&&my_lie<=COLUMN-15)
			  {     
		    	  if(lie_top[my_lie]>lie_top[my_lie-1]+3&&lie_top[my_lie]>lie_top[my_lie-2]+3
					 &&lie_top[my_lie+1]>lie_top[my_lie-1]+3&&lie_top[my_lie+1]>lie_top[my_lie-2]+3)
				  { lie_tiao1=1;}
				  if(lie_top[my_lie-2]>lie_top[my_lie-1]+3&&lie_top[my_lie-2]>lie_top[my_lie]+3
					 &&lie_top[my_lie-3]>lie_top[my_lie-1]+3&&lie_top[my_lie-3]>lie_top[my_lie]+3)
				  { lie_tiao2=1;}
				//  if(diuxian_row-lie_top[my_lie]>20)
				//   {diuxian_dir=0;break;}
			  }
		      if(abs(diuxian_row-lie_top[my_lie])<=3) diuxian_flag=1;
		   }
		   if(My_Road_Condition[diuxian_row].L_position==left_lie&&My_Road_Condition[diuxian_row].R_position==COLUMN-1&&diuxian_row>40)  diuxian_flag=1;
		   if(lie_tiao1==1&&lie_tiao2!=1)  diuxian_dir=0;
		   if(lie_tiao1!=1&&lie_tiao2==1)  diuxian_dir=0;		   
		 }
		 if(diuxian_flag==0) diuxian_dir=0;
		/***************************************单边补线检测************************************************/
		 if(My_Road_Condition[my_row].L_position==left_lie&&my_row<40)  diuxian_left=1;
		 if(My_Road_Condition[my_row].R_position==COLUMN-1&&my_row<40)   diuxian_right=1;
	 	if(danbian_flag==0)
	 	{ 
	 		if(My_Road_Condition[my_row].L_position==left_lie) 
	 	       {danbian_flag=1;danbian_row=my_row;
	 	        road_width=(My_Road_Condition[my_row+1].R_position-My_Road_Condition[my_row+1].L_position)/2;}
	 		if(My_Road_Condition[my_row].R_position==COLUMN-1) 
	 			{danbian_flag=2;danbian_row=my_row;
	 			road_width=(My_Road_Condition[my_row+1].R_position-My_Road_Condition[my_row+1].L_position)/2;}
	 	}
//	 	else if(My_Road_Condition[my_row].R_position<road_width||My_Road_Condition[my_row].L_position+road_width<COLUMN-1) break;
	 }
/***************/	 
	 
	 
	 	
	 	/************************************十字道补线及检测*************************************************/
	    if(left_guaidian1&&left_guaidian2)   
	 	{	for(my_row=left_guaidian_flag[0]-1;my_row>=left_guaidian_flag[1];my_row--)
	 		{
	 		   My_Road_Condition[my_row].L_position=My_Road_Condition[left_guaidian_flag[0]].L_position+
	 				    ((float)left_guaidian_flag[0]-(float)my_row)/((float)left_guaidian_flag[0]-(float)left_guaidian_flag[1])*((float)My_Road_Condition[left_guaidian_flag[1]].L_position-(float)My_Road_Condition[left_guaidian_flag[0]].L_position);
	 		//   Get_extract_image[my_row][My_Road_Condition[my_row].L_position]=255;        //标记
	 		}
	 	}
	    if(right_guaidian1&&right_guaidian2)
	    {    
	    	for(my_row=right_guaidian_flag[0]-1;my_row>=right_guaidian_flag[1];my_row--)
	    	{
	    	 	My_Road_Condition[my_row].R_position=My_Road_Condition[right_guaidian_flag[0]].R_position+
	    	 			((float)right_guaidian_flag[0]-(float)my_row)/((float)right_guaidian_flag[0]-(float)right_guaidian_flag[1])*((float)My_Road_Condition[right_guaidian_flag[1]].R_position-(float)My_Road_Condition[right_guaidian_flag[0]].R_position);
	    	// 	Get_extract_image[my_row][My_Road_Condition[my_row].R_position]=255;        //标记
	    	}
	    } 
	    /*****************************************人字道检测********************************************/
	    if(ren_right_flag==1&&diuxian_left==0&&zhangai_left==0&&ren_hang>20&&ren_done<=ren_num)//&&ren_hang>valid_hang+7  // &&ren_done==0   
	    {
	    	//float offset;
	    	if(ren_hang<55)  
	    	//	if(abs(B_value(carry_row,carry_centre,(byte)(60-ren_hang),&offset))<0.15);
	    	            roadtype=right_ren;
	    }
	    if(ren_left_flag==1&&diuxian_right==0&&zhangai_right==0&&ren_hang>20)//&&ren_hang>valid_hang+7     
	    {	//float offset;
	    	if(ren_hang<55)  
	    	//	if(abs(B_value(carry_row,carry_centre,(byte)(60-ren_hang),&offset))<0.15);
	    	         roadtype=left_ren;
	    }
	    *valid=valid_hang;
	    /*******************************************单边中线拟合************************************************/
	    float x1,x2,y1,y2;
	    if(diuxian_dir==1&&valid_hang>5)
	    {	
	    	y1=diuxian_row;
	    	x1=(My_Road_Condition[diuxian_row].R_position+My_Road_Condition[diuxian_row].L_position)/2;
	    	x2=My_Road_Condition[diuxian_row].L_position;
	    	y2=(lie_top[(byte)x2]+diuxian_row)/2;  
	    	if(x2<left_lie+8)
	    	{
	    		 for(my_lie=x2;(my_lie<left_lie+20)&&(my_lie<My_Road_Condition[diuxian_row].R_position);my_lie++)
	    		 {
	    			if(my_lie<left_lie+2) my_lie=left_lie+2;
	    			if(lie_top[my_lie]>lie_top[my_lie-1]+5&&lie_top[my_lie]>lie_top[my_lie-2]+5)
	    			{ y2=(lie_top[my_lie]+diuxian_row)/2;}
	    		 }
	    	}
	    	if(y2<2) y2=2;
	    	valid_hang=y2-2;	    	
	    	for(my_row=y1;my_row>=y2;my_row--)
	    	{
	    		My_Road_Condition[my_row].centre_position=x1+(y1-my_row)/(y1-y2)*(x2-x1);
	    	}
	    	for(my_lie=My_Road_Condition[diuxian_row].L_position-1;my_lie>left_lie;my_lie--)
	    	{
	    		My_Road_Condition[(byte)y2-1].centre_position=left_lie;
	    		if(Get_extract_image[diuxian_row][my_lie]==0) 
	    			{My_Road_Condition[(byte)y2-1].centre_position=x2;break;}
	    	}
	    }
	    else if(diuxian_dir==2&&valid_hang>5)
	    {	
	    	y1=diuxian_row;
	    	x1=(My_Road_Condition[diuxian_row].R_position+My_Road_Condition[diuxian_row].L_position)/2;
	    	x2=My_Road_Condition[diuxian_row].R_position;
	    	y2=(lie_top[(byte)x2]+diuxian_row)/2;
	    	if(x2>COLUMN-8)
	    	{
	    		for(my_lie=x2;(my_lie>COLUMN-20)&&(my_lie>My_Road_Condition[diuxian_row].L_position);my_lie--)
	    		{
	    			if(my_lie>COLUMN-3) my_lie=COLUMN-3;
	    			if(lie_top[my_lie+1]+5<lie_top[my_lie]&&lie_top[my_lie+2]+5<lie_top[my_lie])
	    			{ y2=(lie_top[my_lie]+diuxian_row)/2;}
	    		}
	    	}
	    	if(y2<2) y2=2;
	    	valid_hang=y2-2;
	    	
	    	for(my_row=y1;my_row>=y2;my_row--)
	    	{
	    		My_Road_Condition[my_row].centre_position=x1+(y1-my_row)/(y1-y2)*(x2-x1);
	    	}
	    	for(my_lie=My_Road_Condition[diuxian_row].R_position+1;my_lie<=COLUMN-1;my_lie++)
	    	{
	    		My_Road_Condition[(byte)y2-1].centre_position=COLUMN-1;
	    		if(Get_extract_image[diuxian_row][my_lie]==0) 
	    			{My_Road_Condition[(byte)y2-1].centre_position=x2;break;}
	    	}
	    }
	    else diuxian_dir=0;
	    
	    if(diuxian_dir==0&&valid_hang<54)
	    {
	    	for(my_row=NEED_ROW-10;my_row>=valid_hang+5;my_row--)
	    	  {
	    		if(My_Road_Condition[my_row].L_position==left_lie) {diuxian_dir=1;diuxian_flag=1;}
	    	         else  {diuxian_dir=0;diuxian_flag=0;break;}
	    	  }
	    	if(diuxian_dir==1)
	    	{
	    		diuxian_row=NEED_ROW-1;
		    	y1=diuxian_row;
		    	x1=(My_Road_Condition[diuxian_row].R_position+My_Road_Condition[diuxian_row].L_position)/2;
		    	x2=left_lie;
		    	y2=(2*valid_hang+NEED_ROW-1)/3;
		    	valid_hang=y2-1;
		    	
		    	for(my_row=y1;my_row>=y2;my_row--)
		    	{
		    		My_Road_Condition[my_row].centre_position=x1+(y1-my_row)/(y1-y2)*(x2-x1);
		    	}
	    	}
	    }
	    if(diuxian_dir==0&&valid_hang<54)
	    {
	    	for(my_row=NEED_ROW-10;my_row>=valid_hang+5;my_row--)
	    	  {
	    		if(My_Road_Condition[my_row].R_position==COLUMN-1)  {diuxian_dir=2;diuxian_flag=1;}
	    	         else  {diuxian_dir=0;diuxian_flag=0;break;}
	    	  }
	    	if(diuxian_dir==2)
	    	{
	    		diuxian_row=NEED_ROW-1;
		    	y1=diuxian_row;
		    	x1=(My_Road_Condition[diuxian_row].R_position+My_Road_Condition[diuxian_row].L_position)/2;
		    	x2=COLUMN-1;
		    	y2=(2*valid_hang+NEED_ROW-1)/3;
		    	valid_hang=y2-1;
		    	
		    	for(my_row=y1;my_row>=y2;my_row--)
		    	{
		    		My_Road_Condition[my_row].centre_position=x1+(y1-my_row)/(y1-y2)*(x2-x1);
		    	}
	    	}
	    }
	    /***************************************************************************************/
	    
	    
	    
	    byte cro_count=0;
	    for(my_row=NEED_ROW-1;my_row>=20;my_row--)
	    {
	    	  if(My_Road_Condition[my_row].R_position==COLUMN-1&&My_Road_Condition[my_row].L_position==left_lie)
	    	   {cro_count++;}
	    }
	    if(cro_count>10)  roadtype=cross_road;
	    /*****************************************起跑线检测********************************************************/
	    byte qipao_flag1=0,qipao_flag2=0;
	    byte qipao_count=0;signed char qipao_panduan;
	    for(my_lie=20;my_lie<=60;my_lie++)
		   {
			  for(my_hang=60;my_hang>20;my_hang--)
			  {	 
				  lie_top[my_lie]=20;
				  if(Get_extract_image[my_hang][my_lie]==255)
			        {
					  //Get_extract_image[my_hang][my_lie]=175;
			          lie_top[my_lie]=my_hang;break;
			        }
			  }
			  if(lie_top[my_lie]>30)  qipao_count++;
			  if(my_lie>=21)
			  {
			     qipao_panduan=lie_top[my_lie]-lie_top[my_lie-1];
			       if(qipao_panduan>=20)   
			    	   qipao_flag1=my_lie;
			    qipao_panduan=lie_top[my_lie-1]-lie_top[my_lie];
				   if(qipao_panduan>=20)   
					  qipao_flag2=my_lie-1;			  
		      }
		   }
	    if(qipao_count>15&&qipao_flag1!=0&&qipao_flag2!=0)  roadtype=stop_road;
	
	    /*****************************************求中线*************************************************/
	    for(my_row=NEED_ROW-1;my_row>valid_hang;my_row--)
	    {
	     /*    if(My_Road_Condition[my_row].L_position==0&&my_row<=danbian_row&&roadtype==normal_road) 
	        	 My_Road_Condition[my_row].centre_position=My_Road_Condition[my_row].R_position-road_width+(danbian_row-my_row)*0.3;
	         else if(My_Road_Condition[my_row].R_position==COLUMN-1&&my_row<=danbian_row&&roadtype==normal_road)
	        	 My_Road_Condition[my_row].centre_position=My_Road_Condition[my_row].L_position+road_width-(danbian_row-my_row)*0.3;
	         else */
	    	if(my_row>diuxian_row)
	    	{	
	    		My_Road_Condition[my_row].centre_position=(My_Road_Condition[my_row].R_position+My_Road_Condition[my_row].L_position)>>1;		 		
	    	    Get_extract_image[my_row][My_Road_Condition[my_row].R_position]=200;
	    	    Get_extract_image[my_row][My_Road_Condition[my_row].L_position]=100;
	    	}
	    	else if(diuxian_dir==0)
	    	{
	    		My_Road_Condition[my_row].centre_position=(My_Road_Condition[my_row].R_position+My_Road_Condition[my_row].L_position)>>1;		 		
	    		Get_extract_image[my_row][My_Road_Condition[my_row].R_position]=200;
	    		Get_extract_image[my_row][My_Road_Condition[my_row].L_position]=100;
	    	}
	    	if(My_Road_Condition[my_row].centre_position>max_right)  My_Road_Condition[my_row].centre_position=max_right;
	    	else if(My_Road_Condition[my_row].centre_position<max_left)  My_Road_Condition[my_row].centre_position=max_left;
	    	carry_centre[NEED_ROW-1-my_row]=My_Road_Condition[my_row].centre_position;
	    	Get_extract_image[my_row][My_Road_Condition[my_row].centre_position]=150;
	    }
}

float qulv(float x1,float y1,float x2,float y2,float x3,float y3)
{
	float s=0,K=0,delta,length;
	s=((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2;
	delta=((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))*(((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)))*(((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2)));
	arm_sqrt_f32(delta,&length);
	K=s/length;
	return(K);
}

