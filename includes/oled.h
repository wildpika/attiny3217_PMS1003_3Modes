#ifndef _oled_H_
#define _oled_H_
#define F_CPU 3333333
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define uint unsigned int
#define uchar unsigned char



void write_spi(uint8_t data);
void OLED_Init(void) ;                             //初始化
void OLED_FILL(uchar bmp_dat);				      //全屏点亮
void OLED_6X8_String(uchar x,uchar y,uchar ch[]);    //ASIIC
void LCD_shuzi(uchar x,uchar y,uchar ch);	    //显示数字
#endif