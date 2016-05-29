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
#define NEED_ROW 	OV7725_EAGLE_H					//��Ҫ�ܹ����������
#define Black_Limit		1

#define COLUMN		    78                //��Ҫ��������ұ�
#define left_lie        0                 //���������
#define VIDEO_CENTER	39					//��������
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
		bool  start_condition;			//�Ƿ���������
		bool  stop_condition;			//�Ƿ��⵽ֹͣ��
		bool  road_condition;			//�Ƿ��⵽��������
		bool  road_Left_flag;
		bool  road_right_flag;
		byte  road_typex;				//  0û�������κ��߼��     1��ʾ���������߼��              2�������߼��              3�������߼��      
		byte  border_distance;			//�������
	    signed char  centre_position;			//����λ��
		byte  L_position;				//�������λ��
		byte  R_position;				//�ұ�����λ��
	
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
