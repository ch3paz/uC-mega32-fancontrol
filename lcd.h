/*
*  file:   lcd.h
*  Author: gguoqing
*
*  Modifications by Joeran Zeller <mx-bounce@gmx.de>:
*    LCD_setPixel added
*    ...see lcd.c for others
*/

#ifndef  __lcd_H__
#define  __lcd_H__

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define uchar unsigned char
#define uint  unsigned int

#define  LCD_RST   PD7
#define  LCD_RD    PD3
#define  LCD_WR    PD4
#define  LCD_RS    PD5
#define  LCD_CS    PD6

#define  LCD_DATA  PORTC

#define  LCD_RST_H  PORTD|= _BV(LCD_RST)
#define  LCD_RST_L  PORTD&=~_BV(LCD_RST)
#define  LCD_RD_H   PORTD|= _BV(LCD_RD)
#define  LCD_RD_L   PORTD&=~_BV(LCD_RD)
#define  LCD_WR_H   PORTD|= _BV(LCD_WR)
#define  LCD_WR_L   PORTD&=~_BV(LCD_WR)
#define  LCD_RS_H   PORTD|= _BV(LCD_RS)
#define  LCD_RS_L   PORTD&=~_BV(LCD_RS)
#define  LCD_CS_H   PORTD|= _BV(LCD_CS)
#define  LCD_CS_L   PORTD&=~_BV(LCD_CS)

#define  TYPE_LCD_DATA    1
#define  TYPE_LCD_COMMAND 0
#define  LCD_SIZE_X     128
#define  LCD_SIZE_Y         160

extern uint colors[];
extern void LCD_Write(uchar type, uint value);
extern void LCD_Write_Data16(uint value);
extern void LCD_setPixel(uint x, uint y, uint colorFG);
extern void Reg_Write(uint reg,uint value);
extern void LCD_SetRamAddr(uint xStart, uint xEnd, uint yStart, uint yEnd);
extern void modify_display_settings(int invertDisplayFlag, int powerDisplayFlag);
extern void LCD_init(int inverseFlag, int powerDisplayFlag);
extern void LCD_clear(uchar n);
extern void LCD_ShowChar(uchar x,uchar y,uint color,uchar ch);
extern void LCD_ShowString(uchar x,uchar y,uint color,char *p);

#endif
