#ifndef _oled_H_
#define _oled_H_
#define F_CPU 3333333
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define uint unsigned int
#define uchar unsigned char



void write_spi(uint8_t data);
void OLED_Init(void) ;                             //��ʼ��
void OLED_FILL(uchar bmp_dat);				      //ȫ������
void OLED_6X8_String(uchar x,uchar y,uchar ch[]);    //ASIIC
void LCD_shuzi(uchar x,uchar y,uchar ch);	    //��ʾ����
#endif