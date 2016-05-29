/*
 * process_Camera_image.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Administrator
 */

#ifndef PROCESS_CAMERA_IMAGE_H_
#define PROCESS_CAMERA_IMAGE_H_
#include"global.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include <math.h>
#include "arm_math.h"
#include"least_squares.h"

#define FALSE_ROW		10
#define COLUMU_diff		4
#define NEED_ROW 	OV7725_EAGLE_H					//需要总共处理的行数
#define Black_Limit		1

#define COLUMN		    78                //需要处理的列右边
#define left_lie        0                 //赛道最左边
#define VIDEO_CENTER	39					//赛道中心
#define  max_left       15
#define max_right       64

#define FIND_ROW		21


#define no_LINE      0x00
#define mid_LINE     0x01
#define left_LINE    0x02
#define right_LINE   0x03;


#ifndef __Road__Contation__
#define __Road__Contation__
typedef struct
{
		bool  start_condition;			//是否有起跑线
		bool  stop_condition;			//是否检测到停止线
		bool  road_condition;			//是否检测到赛道边线
		bool  road_Left_flag;
		bool  road_right_flag;
		byte  road_typex;				//  0没有利用任何线检测     1表示利用中心线检测              2利用左线检测              3利用右线检测      
		byte  border_distance;			//赛道宽度
	    signed char  centre_position;			//中心位置
		byte  L_position;				//左边赛道位置
		byte  R_position;				//右边赛道位置
	
}Road_Conditon;
#endif


typedef enum
{
	normal_road,
	cross_road,
	left_ren,
	right_ren,
	stop_road,
	podao_road
} road_type;

extern byte  Get_extract_image[OV7725_EAGLE_H][OV7725_EAGLE_W];
void  get_second_centre(byte *valid);
void get_edge_image();
void xiaozheng_picture();
void Camera_Data_Convert();

#endif /* PROCESS_CAMERA_IMAGE_H_ */
