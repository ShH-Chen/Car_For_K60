/*
 * OLED_12864.h
 *
 *  Created on: Mar 15, 2014
 *      Author: sheng
 */

#ifndef OLED_12864_H_
#define OLED_12864_H_




#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"


void LCD_Init(void) ;
void LCD_Fill(unsigned char bmp_dat) ;
void LCD_CLS(void);
void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
void LCD_P16x16Ch(unsigned char x,unsigned char y,unsigned char N);
void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
void LCD_P8x16Str(unsigned char x,unsigned char y, char ch[]);
void image_to_12864(void);
void display_num(byte x,byte y,byte num);

#endif /* OLED_12864_H_ */
