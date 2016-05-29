/*
 * speed_pid.h
 *
 *  Created on: Feb 25, 2014
 *      Author: sheng
 */

#ifndef SPEED_PID_H_
#define SPEED_PID_H_

void Init_speedpid(void);
float  PID_speed(float speed);
void CTL_Speed(float upright_duty,float Speed_duty,float DIR_duty);
void set_speed(float speed_value);


#endif /* SPEED_PID_H_ */
