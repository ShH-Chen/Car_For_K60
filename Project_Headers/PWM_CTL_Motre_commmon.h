/*
 * PWM_CTL_Motre_commmon.h
 *
 *  Created on: Feb 18, 2014
 *      Author: Administrator
 */

#ifndef PWM_CTL_MOTRE_COMMMON_H_
#define PWM_CTL_MOTRE_COMMMON_H_

//#define  Fornt    TRUE
#define  Motor_error   0x01
#define PWM_ChannelId0 0X00
#define PWM_ChannelId1 0X01
#define PWM_ChannelId2 0X02
#define PWM_ChannelId3 0X03
#define PWM_ChannelId4 0X04
#define PWM_ChannelId5 0X05
#define PWM_ChannelId6 0X06
#define PWM_ChannelId7 0X07


#ifndef __Motor__direction__
#define	__Motor__direction__
typedef enum
{
	Fornt,
	Back
}Motor_direction;
#endif


#ifndef __car__walk__direction__
#define __car__walk__direction__
typedef enum
{
	dir_left,
	dir_right,
	dir_straight	
}car_walk_direction;
#endif




void Init_FTM0();
void  Set_left_direction(Motor_direction  direction);
void Set_right_direction(Motor_direction  direction);
unsigned short Set_left_dutycycle(int dutycycle);
unsigned short Set_right_dutycycle(int dutycycle);
unsigned short Set_common_dutycycle(int dutycycle);
unsigned short CTL_left_Motor(int dutycycle);
unsigned short CTL_right_Motor(int dutycycle);

#endif /* PWM_CTL_MOTRE_COMMMON_H_ */
