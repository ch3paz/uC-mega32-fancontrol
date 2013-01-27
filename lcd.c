/*
*  File:   lcd.c
*  Author: gguoqing
*
*  Modifications by Joeran Zeller <mx-bounce@gmx.de>:
*   - LCD_setPixel added
*   - LCD_ShowString now with \n support
*   - LCD_init invers feature added
*   - LCD_init -> can now put LCD off (not the backlight!)
*   - Code reformatted, saved some memory through deleting something ;)
*   - FREQ adjusted
*   - Some #defines for fonthandling added
*   - colors[] modified
*/

#include "lcd.h"
#include "5x8_vertikal_LSB_1.h"
#include <stdio.h>

#define FREQ F_CPU
#define CHAR_W 8//8   /* Original Font -> 8*16 */
#define CHAR_H 5//16  /* CHAR_W/H swapped due to other font and
                      /* rotating the display 90 degrees */
#define LINE_SPACE -3 /* If font don't fill CHAR_H, correct it here */
#define FONTOFFSET 0  /* Depends on codepage/fonttype */
#define FONTARRAYNAME font

/* Colors --> colors[0...] */
uint colors[]={
  0xf800, /* red        */
  0xf81f, /* lilac      */
  0xffff, /* white      */
  0x0000, /* black      */
  0x07e0, /* green 1    */
  0x03e0, /* green 2    */
  0x09a0, /* green 3    */
  0xF4A1, /* yellow 1   */
  0xF3C7, /* yellow 2   */
  0xFAAC, /* yellow 3   */
  0x11BB  /* FIXME give color name */
};

/* Command for writing to display (16Bit) --> 0: command 1: data */
void LCD_Write(uchar type, uint value){
  if(type == 0)
    LCD_RS_L;
  else
    LCD_RS_H;

  LCD_WR_L;
  LCD_DATA = (uchar)((value>>8)&0xff);
  LCD_WR_H;

  LCD_WR_L;
  LCD_DATA = (uchar)(value&0xff);
  LCD_WR_H;
}

/* Write 16-bit data to display */
void LCD_Write_Data16(uint value){
  LCD_CS_L;
  LCD_RS_H;

  LCD_WR_L;
  LCD_DATA = (uchar)((value>>8)&0xff);
  LCD_WR_H;

  LCD_WR_L;
  LCD_DATA = (uchar)(value&0xff);
  LCD_WR_H;
  LCD_CS_H;
}

/* Set one pixel with: PosX, PosY, ColorFG */
void LCD_setPixel(uint x, uint y, uint colorFG){
  /* Set startadress, inverted y,x here for "top->down"-display */
  LCD_SetRamAddr(y, y, x, x);
  LCD_Write_Data16(colors[colorFG]); /* draw pixel with color colorFG */
}

/* Write data to register */
void Reg_Write(uint reg,uint value){
  LCD_Write(TYPE_LCD_COMMAND,reg);
  LCD_Write(TYPE_LCD_DATA,value);
}

/* Set startadress of drawing */
void LCD_SetRamAddr(uint xStart, uint xEnd, uint yStart, uint yEnd){
  uint VerPos, HorPos, StartAddr;

  LCD_CS_L;
  HorPos    = (uint)((xEnd<<8)|xStart);
  VerPos    = (uint)((yEnd<<8)|yStart );
  StartAddr = (uint)((yStart<<8)|xStart);

  Reg_Write(0x16, HorPos);
  Reg_Write(0x17, VerPos);

  Reg_Write(0x21, StartAddr);         // 0x21
  LCD_Write(TYPE_LCD_COMMAND,0x22);   // 0x22
  LCD_CS_H;
}

/* LCD init */
void  LCD_init(int invertDisplayFlag, int powerDisplayFlag){
  LCD_RST_L;
  _delay_ms(20);
  LCD_RST_H;
  _delay_ms(20);

  LCD_CS_H;
  LCD_RS_L;
  LCD_RD_H;
  LCD_WR_H;
  LCD_CS_L;
  _delay_ms(10);

  /* NOTICE:
   *
   * I think we can save power here with the "power control" registers.
   * But i don't messed around with them so far.
   *
   */
  Reg_Write(0x0007,0x0100); // Display control GON=0,DTE=0,D1=0,D0=0,
  Reg_Write(0x000d,0x0000); // Power control 4  PON=0
  Reg_Write(0x0000,0x0001); // OSC Start
  _delay_ms(20);
  Reg_Write(0x000e,0x2000); // Power control 5 VCOMG=1
  Reg_Write(0x000c,0x0001); // Power contorl 3 VCI=VDD , VCI1=VCI*1
  Reg_Write(0x000d,0x0003); // Power control 4 VLCD
  Reg_Write(0x0004,0x0000); // Power control 2 CAD=0
  Reg_Write(0x000d,0x0903); // Power control 4 VREG2OUT=-5.5*VCI
  Reg_Write(0x0003,0x0414); // Power Control 1
  Reg_Write(0x000e,0x1212); // Power control 5  Vcom
  _delay_ms(50);            // VCOMH(3V~VREG1OUT) , VCOML(0.5~1V)

  Reg_Write(0x000d,0x031d); // 0x318  // Power control 4  PON=1  VRL3~VRL0=1001 -->VREG2OUT=-5.5*VCI
  // VRH3~VRH0=0011 -->VREG1OUT=1.65*REGP=4.62V , REGP=VCI1

  Reg_Write(0x0001,0x0113); // Driver output contorl SM,GS,SS NL4~NL0=10011(1/176Duty)
  Reg_Write(0x0002,0x0700); // LCD-Driving-waveform control  FLD1 FLD0 B/C EOR NW5~NW0=001101
  Reg_Write(0x0005,0x1030); // Entry Mode RGB MODE , I/D1 I/D0=11
  Reg_Write(0x0006,0x0000);
  Reg_Write(0x000b,0x4001);
  Reg_Write(0x000f,0x0000); // GATE SCAN POSITION  G17 Start scanf G17->G176
  Reg_Write(0x0011,0x0000);
  Reg_Write(0x0014,0x9F00);
  Reg_Write(0x0015,0x0000);
  Reg_Write(0x0016,0x7F00); // Horizontal window address  128
  Reg_Write(0x0017,0x9F00); // Vertical window Address  160
  Reg_Write(0x0021,0x0000);

  Reg_Write(0x0030,0x0000);
  Reg_Write(0x0031,0x0204);
  Reg_Write(0x0032,0x0302);
  Reg_Write(0x0033,0x0000);
  Reg_Write(0x0034,0x0504);
  Reg_Write(0x0035,0x0405);
  Reg_Write(0x0036,0x0707);
  Reg_Write(0x0037,0x0100);
  Reg_Write(0x003a,0x1506);
  Reg_Write(0x003b,0x000f);
  Reg_Write(0x0007,0x0005);
  Reg_Write(0x0007,0x0025);
  Reg_Write(0x0007,0x0027);

  modify_display_settings(invertDisplayFlag, powerDisplayFlag);

  LCD_Write(TYPE_LCD_COMMAND,0x22);   // 0x22
  _delay_ms(10);
  LCD_CS_H;
}

/* NOTICE: Init Display in normal or invers.
 *         Power off display (NOT the backlight!)
 */
void modify_display_settings(int invertDisplayFlag, int powerDisplayFlag){
  if ((invertDisplayFlag == 0) && (powerDisplayFlag == 0)){
    Reg_Write(0x0007,0x0135); // normal, power off (00)
  }
  else if ((invertDisplayFlag == 1) && (powerDisplayFlag == 0)){
    Reg_Write(0x0007,0x0131); // invers, power off (10)
  }
  else if ((invertDisplayFlag == 1) && (powerDisplayFlag == 1)){
    Reg_Write(0x0007,0x0133); // invers, power on (11)
  }
  else{
    Reg_Write(0x0007,0x0137); // normal, power on (01)
  }
}

/* Clearscreen (with color, uchar n <-- color 0-7) */
void  LCD_clear(uchar n){
  uint num;
  LCD_SetRamAddr(0, 127, 0, 159);

  for(num = 20480; num > 0; num--){ /*160*128=20480 */
      LCD_Write_Data16(colors[n]);
  }
}

/* Show single char */
void LCD_ShowChar(uchar x, uchar y, uint color, uchar ch){
  uchar temp;
  uchar pos, t;

  if(x > (LCD_SIZE_Y-CHAR_W) || y>(LCD_SIZE_X-CHAR_H))

  return;

  /* Set startadress, inverted y,x here for "top->down"-display */
  LCD_SetRamAddr(y, y + CHAR_W - 1, x, x + CHAR_H - 1);
  ch = ch - FONTOFFSET;

  for(pos = 0; pos < CHAR_H; pos++){
    temp = pgm_read_byte(&FONTARRAYNAME[ch][pos]);

    for(t = 0; t < CHAR_W; t++){
      if(temp&0x80)
        LCD_Write_Data16(colors[color]);  //textcolor
      else
        LCD_Write_Data16(0xffff);  //backcolor
      temp<<=1;
    }
  }
}

/* Show string --> x,y: startposition, *p: string address */
void LCD_ShowString(uchar x,uchar y,uint color, char *p){
  while(*p != '\0'){
    if (*p == '\n'){
      y += CHAR_H-LINE_SPACE;
      x = 0;
      p++;
    }
    else{
      LCD_ShowChar(x, y, color, *p);
      x += CHAR_W;
      p++;
    }
  }
}
