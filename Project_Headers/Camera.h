/*
 * Camera.h
 *
 *  Created on: Feb 18, 2014
 *      Author: sheng
 */

#ifndef CAMERA_H_
#define CAMERA_H_



void InitCamera(void);
//void ReadCamera(void);
void Init_DMA1(void);
#define ARR_SIZE(a)   (sizeof((a))/sizeof( ( (a)[0] ) ) )
//void CameraDataConvert(void);


#endif /* CAMERA_H_ */
