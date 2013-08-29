/*
 * Copyright (c) 2010-2011 by Joeran Zeller <mx-bounce@gmx.de>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * Description of project:
 *
 *  1 or 2 fans should be take action if:
 *   - the absolute humidity of air outside is $foo less than inside
 *   - there is a given minimum temperature inside
 *   - there is a given relative humidity inside
 *
 *  Also displays and logs temperature, humidity (relative and
 *  absolute), dewpoint and light (day/night, just for the log) to an
 *  SD-card and a coloured LCD.
 *
 *  I'll connect RX/TX to an X-Port, so there are some "Networkcommands"
 *  to get/set different values on the fly.
 *
 * Description of accepted commands:
 *
 *  "ping"                  - uC returns "pong" ;)
 *  "set.server"            - toggles a relais connected to PA5 (has
 *                            nothing to do with the fancontrol)
 *  "get.data:$lastLines"   - uC sends $lastlines (int) logged to the SD
 *                            (e.g.: 'get:0' for all records, 'get:7'
 *                            for the last 7 records...)
 *  "get.values"            - uC returns the current active values
 *  "set.$foo:$bar"         - sets new values to uC. Possible values for
 *                            $foo are:
 *                            set.rhmin:$bar     (double, in %)
 *                            set.hadiff:$bar    (double, in g)
 *                            set.tmin:$bar      (double, in C)
 *                            set.deltat:$bar    (double, in C)
 *                            set.deltarh:$bar   (double, in %)
 *                    NOTICE: Smallest delay possible is 30s, everything
 *                            else is multiple of 30s.
 *                            set.delay:$bar     (int, in s (s/30!) )
 *                            set.disp.bklt:$bar (int, 0/1)
 *                            set.disp.pwr:$bar  (int, 0/1)
 *                            set.disp.inv:$bar  (int, 0/1)
 *
 * Description of wireing/hardware:
 *
 *  - Atmega32@3.6864Mhz (we want fast baudrates)
 *  - Sensirion SHT75 (two of them --> inside/outside)
 *  - SD-card-slot for logging
 *  - LCD-display with 128x160 dots/a lot of colors ;)
 *  - one LDR
 *  - some switches (to eject SD, display toggle..., reset)
 *  - copper, solder, resistors, coffee, time, coffee, time++, cof... :)
 *
 *  SHT75 connected to:               LCD-display:
 *    PB0 - Clk (both sensors)          PD3 - /RD
 *    PB1 - Data (Sensor 1)             PD4 - /WR
 *    PB2 - Data (Sensor 2)             PD5 - RS
 *                                      PD6 - /CS
 *  SD-Card-Slot:                       PD7 - /RST
 *    PB4 - CS
 *    PB5 - DI                          PC0 - D15
 *    PB6 - DO                          PC1 - D14
 *    PB7 - Clk                         PC2 - D13
 *                                      PC3 - D12
 *                                      PC4 - D11
 *                                      PC5 - D10
 *                                      PC6 - D09
 *                                      PC7 - D08
 *
 *  Switch:
 *    PA7 - Switches pull up PA7 over $foo Ohm
 *
 *  LDR:
 *    PA6 - Just to see if it's day or night.
 *
 *  Relais:
 *    PB3 - Toggle fan(s)
 *    PA5 - N/A (toggles whatever i connect to it ;) )
 *
 *  Backlight:
 *    PA4 - Backlight, switched over a transistor.
 *
 *
 * NOTICE:
 *
 *  I'm using a 1GB card with FAT16 - FAT32 can be enabled but it's
 *  to much bloat for the Mega32.
 *
 *  I use keywords like FIXME or NOTICE 'during programming. If you
 *  find FIXME - fix it ;) On NOTICE it's up to you if you'll read
 *  or ignore it.
 */

#define TRUE 1
#define FALSE 0
#define DEBUG TRUE

/* Defaults for default_value_init() */
#define HADIFF 1.0
#define RHMIN 82.5
#define TMIN 15.0
#define DELTAT 0.3
#define DELTARH 1.0
#define DELAY 20 /* --> 20*30s=600s , look at ISR/main() */

/* Colors for LCD */
#define RED 0
#define LILAC 1
#define WHITE 2
#define BLACK 3
#define GREEN1 4
#define GREEN2 6
#define GREEN3 5
#define YELLOW1 7
#define YELLOW2 9
#define YELLOW3 8
#define BLUE 10

#define FILENAME "log.csv" /* Filename written to SD */
#define BUTTONPIN 7        /* ADC pin, connection of Ejectswitch */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
//#include <util/delay.h>

#include "analog.h"
#include "sht75.h"
#include "usart.h"
#include "lcd.h"

#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"

/* rrfs - thanks Roland, very nice work! */
struct fat_fs_struct* fs;
struct fat_dir_struct* dd;
struct fat_file_struct* fd;
struct fat_dir_entry_struct file_entry;
struct partition_struct* partition;
struct fat_dir_entry_struct directory;
static uint8_t find_file_in_dir(
                                  struct fat_fs_struct* fs,
                                  struct fat_dir_struct* dd,
                                  const char* name,
                                  struct fat_dir_entry_struct* dir_entry
                                );
static struct fat_file_struct* open_file_in_dir(
                                    struct fat_fs_struct* fs,
                                    struct fat_dir_struct* dd,
                                    const char* name
                                                );

/* My stuff */
void read_sensors(uint8_t sensorSelect);
void draw_screen(uint8_t drawTextOnlyFlag);
void port_init();
void open_fs();
void open_file();
void write_data_to_file(uint8_t sensorSelect);
void close_file();
void init_timer();
void _delay_s(uint8_t delay);
void check_fan(double haIn, double haOut, double rhIn,
               double tempIn, double tempOut);
void check_uart();
void check_button_pressed(uint8_t doLogging, uint8_t powerDisplayFlag,
                          uint8_t invertDisplayFlag,
                          uint8_t powerDisplayBacklightFlag);
void default_value_init();
void send_data(uint32_t lastLines);
uint8_t check_invert_display();
uint8_t check_power_display();
uint8_t check_power_backlight_display();
uint8_t check_logging();
double cut_token();

#if DEBUG
  void error_report(uint8_t err);
#endif

/* Min values for prevent fans running if they are not applicable.
 *
 * NOTICE:
 *    They're changeable over RS232/Network. So they are doubles and
 *    ints, not #defines.
 *
 * Values are initialized to defaults in default_value_init().
 */
typedef struct min_valuesst{
  double min_ha_diff;/* Difference of absolut humidity should be */
  double min_rh;     /* Min humidityIn to run fans should be, in % */
  double min_temp;   /* Min tempIn should always be */
  double deltaT;     /* Run fans only when tempIn > min_temp+deltaT */
  double deltaRH;    /* Run fans only when min_rh > min_rh+deltaRH */
  uint16_t sensor_delay; /* Delay in seconds */
  uint8_t send_data;     /* 0 don't send, 1 send */
} values_array;

/* Struct for the fan(s) */
typedef struct fanst{
  double haIn;
  double haOut;
  double rhIn;
  double tempIn;
  double tempOut;
  uint8_t running;
} fan_array;

/* Struct for the graph */
typedef struct graphst{
  uint8_t x_pos;
  uint16_t y_brightness;
  int8_t y_temp;
  int8_t y_humidity;
  int8_t y_dewpoint;
} graph_array;

values_array values;
fan_array fan;
graph_array graph;
sht75_array sht75;

uint8_t sht75_status;
uint8_t sht75_error;

volatile uint8_t doLogging = TRUE;          /* Should we log or should we go now...*/
volatile uint8_t invertDisplayFlag = FALSE; /* Invert display or not               */
volatile uint8_t powerDisplayFlag = TRUE;   /* Poweroff display, NOT the backlight */
volatile uint8_t powerDisplayBacklightFlag = TRUE; /* Poweroff backlight only      */
volatile uint8_t drawTextOnlyFlag = FALSE;  /* Update only the text, not the graph */
volatile uint8_t buttonCheckFlag = FALSE;   /* Check every 5s if button is pressed */
volatile uint8_t uartCheckFlag = FALSE;     /* Is set if something is received     */
volatile uint8_t cntr_isr = 0;   /* Counting from 1 to 5 (5 == 30s gone) */
volatile uint8_t cntr_graph = 0; /* For updateing the graph (++ every 1s)*/
uint8_t sensorSelect = 1;        /* Sensor 1,2(...3,4,5,)                */
uint8_t firstCycleFlag = TRUE;   /* To override deltaT the first time    */
uint32_t lastLines = 0;  /* How many records should be send? Default all */
char mp_buffer[45];      /* Multipurpose buffer for stringoperations     */
char uart_rcv[45];       /* Receive buffer for incoming data             */
char header[] = "Ti;RHi;AHi;DPi;To;RHo;AHo;DPo;Br.;Fan\n";

int main(){
  /* Wait 1500ms, let all electrons come down :)
   * (sht75 and sd _eventually_ need this)
   */
  //_delay_ms(1500);

  /* Set up everything */
  port_init();
  uart_init();
  adcInit();
  ShtInit(1);
  ShtInit(2);
  sei();
  init_timer();
  LCD_init(invertDisplayFlag, powerDisplayFlag);
  LCD_clear(WHITE);
  open_fs();
  open_file(FILENAME);
  default_value_init();
  graph.x_pos = 0;

  while(TRUE){

    /* Button to remove sd pressed? */
    while (!doLogging){
      close_file();
      LCD_clear(WHITE);
      LCD_ShowString(20, 64, RED, "SD card ejected");
      _delay_s(5);
    }

    /* Logging all the way... */
    while (doLogging){

      /* Update only text every 30s.
       * NOTICE: LCD_ShowString(*#) --> Just to send an
       * "I'm alive" message.
       */
      if (cntr_isr == 2){
        LCD_ShowString(72, 8, BLUE, "* ");
      }
      if (cntr_isr == 5){
        LCD_ShowString(72, 8, BLUE, " *");
        for (sensorSelect = 1; sensorSelect <=2; sensorSelect++){
          read_sensors(sensorSelect);
          drawTextOnlyFlag = TRUE;
          draw_screen(drawTextOnlyFlag);
          drawTextOnlyFlag = FALSE;
        }
        cntr_isr = 0;
        cntr_graph++;
      }

      /* Update everything (text AND graph),
       * check conditions for running fans,
       * write data to file.
       */
      if (cntr_graph == values.sensor_delay){
        for (sensorSelect = 1; sensorSelect <=2; sensorSelect++){
          read_sensors(sensorSelect);
          draw_screen(drawTextOnlyFlag);
          write_data_to_file(sensorSelect);
        }
        check_fan(fan.haIn, fan.haOut, fan.rhIn, fan.tempIn, fan.tempOut);
        cntr_graph = 0;
      }

      /* Moved "button pressed?" checks from OVF_ISR to main(),
       * checks are done every 5s
       */
      if (buttonCheckFlag == TRUE){
        check_button_pressed(doLogging, powerDisplayFlag,
          invertDisplayFlag, powerDisplayBacklightFlag);
        buttonCheckFlag = FALSE;
      }

      /* Moved UART-Stuff from USART_RXC to main() */
      if (uartCheckFlag == TRUE){
        strcpy(mp_buffer, uart_rcv);
        check_uart();
        uartCheckFlag = FALSE;
      }
    }
  }
 return 0;
} /* main end - and YES, it's a stupid comment ;) */

/* +-------------------------------------------------------------------+
 * |<<<<<<<<<<<<<<<<<<<<<<<<< BEGIN FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>|
 * +-------------------------------------------------------------------+
 */

/* Read sensors, sht75s and brightness */
void read_sensors(uint8_t sensorSelect){

  ShtReset(sensorSelect);
  ShtReadEverything(&sht75, sensorSelect);

  /* Brightness needed once, only for the record.
   * Also grab HAin/out, RHin, TempIn for checking fans.
   */
  if (sensorSelect == 1){
    fan.haIn = sht75.AbsolutHumidity;
    fan.rhIn = sht75.Humidity;
    fan.tempIn = sht75.Temperature;
    graph.y_brightness = adcReadOnce(6)/10;
  }
  else{
    fan.tempOut = sht75.Temperature;
    fan.haOut = sht75.AbsolutHumidity;
  }

  /* Check for error in sht75 */
  if (ShtReadStatus(&sht75_status, sensorSelect) != FALSE){
    LCD_clear(WHITE);
    LCD_ShowString(7, 64, RED,"Sensor failure");
    _delay_s(10);
    LCD_clear(RED);
  }
}

/* Drop values to Display */
void draw_screen(uint8_t drawTextOnlyFlag){
  /* Text should be more often updated than the graph */
  if (drawTextOnlyFlag == FALSE){
    /* Draw graph, display is 128x160. I'm useing it as 160x128. */
    if(graph.x_pos < 160){
      /* Get values "between $foo and $bar" for the display,
       * made it more "sensible" (foo*1.3)
       */
      graph.y_temp = sht75.Temperature; // e.g. foo*1,25
      graph.y_humidity = sht75.Humidity;
      graph.y_dewpoint = sht75.Dewpoint;

      if (sensorSelect == 1){
        /* NOTICE
         * Delete pixels before writeing new ones.
         * Hided here because we want to delete them only once at the
         * first call of draw_screen()!
         */

        /* Delete pixels if graph has gone down into textarea */
        for (int i = 0; i <= 24; i++){
          LCD_setPixel(graph.x_pos, i, WHITE);
        }

        /* Vertikal marker line */
        for (int i = 25; i <= 116; i++){
          LCD_setPixel(graph.x_pos, i, WHITE);
          LCD_setPixel(graph.x_pos+1, i, WHITE);
          LCD_setPixel(graph.x_pos+2, i, RED);

          /* Horizontal "legs" on top and bottom of vertikal marker */
          if (i == 25){
            for (int j = graph.x_pos+3; j < graph.x_pos+6; j++){
              LCD_setPixel(j, i, RED);
              LCD_setPixel(j, i+91, RED);
            }
          }
        }
        /* "Offset" the both datalines.
         * x,y in LCD_setPixel is !!INVERTED THERE!!
         */
        LCD_setPixel(graph.x_pos, graph.y_temp+55, YELLOW1);
        LCD_setPixel(graph.x_pos, graph.y_humidity-10, YELLOW2);
        /* NOTICE: Think i don't need the dewpoint on display.
         * LCD_setPixel(graph.x_pos, graph.y_dewpoint+70, YELLOW3);
         */
      }
      else{
        LCD_setPixel(graph.x_pos, graph.y_temp+55, GREEN1);
        LCD_setPixel(graph.x_pos, graph.y_humidity-10, GREEN2);
        /* NOTICE: Think i don't need the dewpoint on display.
         * LCD_setPixel(graph.x_pos, graph.y_dewpoint+70, GREEN3);
         */

        /* Draw a "fan is running"-timeline on the display*/
        if (fan.running == 1){
          LCD_setPixel(graph.x_pos, 116, BLACK);
        }

        /* Draw a "so much light"-line on the display */
        LCD_setPixel(graph.x_pos, graph.y_brightness, LILAC);

        graph.x_pos++;
      }
    }
    else{
      graph.x_pos = 0;
    }
  }
  /* Update text on every call */
  switch (sensorSelect){
    case 1 :    sprintf(mp_buffer, "%4.1f g/m3", sht75.AbsolutHumidity);
                LCD_ShowString(0, 117, YELLOW3, mp_buffer);
                sprintf(mp_buffer, "T %5.1fC", sht75.Temperature);
                LCD_ShowString(0, 0, YELLOW1, mp_buffer);
                sprintf(mp_buffer, "RH%5.1f%%", sht75.Humidity);
                LCD_ShowString(0, 8, YELLOW2, mp_buffer);
                sprintf(mp_buffer, "DP%5.1fC", sht75.Dewpoint);
                LCD_ShowString(0, 16, YELLOW3, mp_buffer);
                break;
    case 2 :    sprintf(mp_buffer, "%4.1f g/m3", sht75.AbsolutHumidity);
                LCD_ShowString(80, 117, GREEN3, mp_buffer);
                sprintf(mp_buffer, "T %5.1fC", sht75.Temperature);
                LCD_ShowString(95, 0, GREEN1, mp_buffer);
                sprintf(mp_buffer, "RH%5.1f%%", sht75.Humidity);
                LCD_ShowString(95, 8, GREEN2, mp_buffer);
                sprintf(mp_buffer, "DP%5.1fC", sht75.Dewpoint);
                LCD_ShowString(95, 16, GREEN3, mp_buffer);
                sprintf(mp_buffer, "%i", graph.y_brightness);
                LCD_ShowString(72, 0, WHITE, "   ");
                LCD_ShowString(72, 0, LILAC, mp_buffer);
                break;
  }
}

/* Set up ports (direction, pullup).
 * See sht75.c, manipulates them too.
 */
void port_init(void){
  DDRA = 0x00;  /* All input */
  DDRA |= (1<<DDA5) | (1<<DDA4); /* PA4,5 as output */
  /* All with pullup, PA4,5 is HIGH (LCD Backlight on, Relais off) */
  PORTA |= (1<<PA0) | (1<<PA1) | (1<<PA2) | (1<<PA3) | (1<<PA4) |
           (1<<PA5) | (1<<PA6) | (1<<PA7);

  DDRB = 0xFF; /* All output */
  DDRB &= ~(0<<DDB4) | (0<<DDB5) | (0<<DDB7); /* PB4,5,7 as input */
  PORTB |= (1<<PB4) | (1<<PB5) | (1<<PB7); /* Pull-Up on PB4,5,7 */

  DDRC = 0xFF;  /* All output */
  PORTC = 0xFF; /* All high */

  DDRD = 0xFF;  /* All output */
  PORTD = 0xFF; /* All high */
}

/* Throw only a errno, saves space (i hope...)
 * ErrorNumberCount is now:
 * >>> 15 <<<
 */
#if DEBUG
  void error_report(uint8_t err){
    char errep[3];
    sprintf(errep, "E: %i", err);
    LCD_ShowString(10, 120, RED, errep);
    uart_sendstring(errep);
    _delay_ms(500);
  }
#endif

/* Open filesystem */
void open_fs(){

  /* setup sd card slot */
  if(!sd_raw_init()){
    #if DEBUG
      error_report(1);
    #endif
  }

  /* Open first partition */
  partition = partition_open(
                             sd_raw_read,
                             sd_raw_read_interval,
                             #if SD_RAW_WRITE_SUPPORT
                               sd_raw_write,
                               sd_raw_write_interval,
                             #else
                               0, 0,
                             #endif
                             0
                            );

  if(!partition){
  /* If the partition did not open, assume the storage device
   * is a "superfloppy", i.e. has no MBR.
   */
    partition = partition_open(
                  sd_raw_read,
                  sd_raw_read_interval,
                  #if SD_RAW_WRITE_SUPPORT
                  sd_raw_write,
                  sd_raw_write_interval,
                  #else
                  0, 0,
                  #endif
                  -1
                  );
    if(!partition){
      #if DEBUG
        error_report(2);
      #endif
    }
  }

  /* Open file system */
  fs = fat_open(partition);

  if(!fs){
    #if DEBUG
      error_report(3);
    #endif
  }

  /* Open root directory */
  fat_get_dir_entry_of_path(fs, "/", &directory);
  dd = fat_open_dir(fs, &directory);

  if(!dd){
    #if DEBUG
      error_report(4);
    #endif
  }
}

/* Open file, create if not exist */
void open_file(){
  fd = open_file_in_dir(fs, dd, FILENAME);
  if(!fd){
    #if DEBUG
      error_report(5);
    #endif

    /* File not found, create new one */
    if(!fat_create_file(dd, FILENAME, &file_entry)){
      #if DEBUG
        error_report(6);
      #endif
    }
    else{
      sd_raw_sync();
    }

    fd = open_file_in_dir(fs, dd, FILENAME);
    if(!fd){
      #if DEBUG
        error_report(7);
      #endif
    }
  }

  /* File exists, move to EOF */
  if(fd){
    int32_t write_offset = 0;

    if(!fat_seek_file(fd, &write_offset, FAT_SEEK_END)){
      #if DEBUG
        error_report(8);
      #endif

      fat_close_file(fd);
    }
    else{
      /* Append header to file (only into the first line) */
      if (write_offset == 0){

        if(fat_write_file(fd, (uint8_t*) header, strlen(header)) !=
          strlen(header)){
            #if DEBUG
              error_report(9);
            #endif
        }
      }
      sd_raw_sync();
    }
  }
}

/* Write all the data to file */
void write_data_to_file(uint8_t sensorSelect){
  /* Append data to file */
  if (sensorSelect == 1){
    sprintf(mp_buffer, "%02.1f;%02.1f;%02.1f;%02.1f;",
          sht75.Temperature,
          sht75.Humidity,
          sht75.AbsolutHumidity,
          sht75.Dewpoint
          );
  }
  else{
    sprintf(mp_buffer, "%02.1f;%02.1f;%02.1f;%02.1f;%i;%i\n",
          sht75.Temperature,
          sht75.Humidity,
          sht75.AbsolutHumidity,
          sht75.Dewpoint,
          graph.y_brightness,
          fan.running
          );
  }

  if(fat_write_file(
                    fd,
                    (uint8_t*) mp_buffer,
                    strlen(mp_buffer)
       ) != strlen(mp_buffer)
     ){
        #if DEBUG
          error_report(10);
        #endif
      }
  sd_raw_sync();
}

/* Stop loggin to SD if button 1 is pressed so it can be removed */
uint8_t check_logging(){
  int buttonValue = 0;

  /* Get an average value from the button */
  for (int i = 0; i< 3; i++){
    buttonValue += adcReadOnce(BUTTONPIN);
    buttonValue = buttonValue/3;
  }

  if ((buttonValue >= 185) && (buttonValue <= 205)){
    return FALSE;
  }
  else{
    return TRUE;
  }
}

/* Close file - no return, no recover, only reset to restart ;) */
void close_file(){
  sd_raw_sync();
  fat_close_file(fd);
  fat_close(fs);
  partition_close(partition);
}

uint8_t find_file_in_dir(
                         struct fat_fs_struct* fs,
                         struct fat_dir_struct* dd,
                         const char* name,
                         struct fat_dir_entry_struct* dir_entry){
    while(fat_read_dir(dd, dir_entry)){
        if(strcmp(dir_entry->long_name, name) == 0){
          fat_reset_dir(dd);
          return 1;
        }
    }

  return 0;
}

struct fat_file_struct* open_file_in_dir(
                                         struct fat_fs_struct* fs,
                                         struct fat_dir_struct* dd,
                                         const char* name){
  struct fat_dir_entry_struct file_entry;

  if(!find_file_in_dir(fs, dd, name, &file_entry))
    return 0;

    return fat_open_file(fs, &file_entry);
}

/* Now i know why so many people implement theire own _delay_x... :) */
void _delay_s(uint8_t delay){
  for (int yaled = 0; yaled < delay; yaled++){
    _delay_ms(1000);
  }
}

void check_uart(){

  char lf[] = "\n";

  /* Useless use of memory :) */
  if (strncmp(mp_buffer, "ping", 4) == 0){
    uart_sendstring("pong\n");
  }

  /* Has nothing to do with the fancontrol, just toggle
   * the second relais
   */
  else if (strncmp(mp_buffer, "set.server", 10) == 0){
    uart_sendstring("Server toggled\n");
    PORTA ^= (1<<PA5);
  }

  /* Read data from SD
   * e.g.:
   * "get:0"  for everything
   * "get:20" for last 20 lines
   * "get:XX" for last XX lines...
   *
   * mp_buffer messed up in cut_token(), so don't get fooled.
   */
  else if (strncmp(mp_buffer, "get.data:", 9) == 0){
    lastLines = (int) cut_token();
    uart_sendstring("Sending data...\n");
    uart_sendstring("Lines: ");
    uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
    //uart_sendstring("\n");
    send_data(lastLines);
  }

  /* Throw currently active values to UART */
  else if (strncmp (mp_buffer, "get.values", 10) == 0){
    sprintf(mp_buffer, "%02.1f", values.min_ha_diff);
    uart_sendstring("Minimum absolute humidity difference: ");
    uart_sendstring(mp_buffer); uart_sendchar(*lf);
    sprintf(mp_buffer, "%02.1f", values.min_rh);
    uart_sendstring("Minimum relative humidity indoor: ");
    uart_sendstring(mp_buffer); uart_sendchar(*lf);
    sprintf(mp_buffer, "%02.1f", values.min_temp);
    uart_sendstring("Minimum temperature indoor: ");
    uart_sendstring(mp_buffer); uart_sendchar(*lf);
    sprintf(mp_buffer, "%02.1f", values.deltaT);
    uart_sendstring("DeltaT: ");
    uart_sendstring(mp_buffer); uart_sendchar(*lf);
    sprintf(mp_buffer, "%02.1f", values.deltaRH);
    uart_sendstring("DeltaRH: ");
    uart_sendstring(mp_buffer); uart_sendchar(*lf);
    sprintf(mp_buffer, "%i", values.sensor_delay*30);
    uart_sendstring("Sensor delay: ");
    uart_sendstring(mp_buffer); uart_sendchar(*lf);
  }
/* NOTICE:
 *  I don't check if given value make sense.
 *  So REMEMBER: If the returned value differs from the given one
 *  something went wrong!
 *
 *  mp_buffer is messed up in cut_token(), so don't get fooled.
 */

  /* Catch new active values --> Set min. humidity difference */
  else if (strncmp(mp_buffer, "set.hadiff:", 11) == 0){
    values.min_ha_diff = cut_token();
    uart_sendstring("HAmin set to: "); uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
  }

  /* Catch new active values --> Set min. relative humidity */
  else if (strncmp(mp_buffer, "set.rhmin:", 10) == 0){
    values.min_rh = cut_token();
    uart_sendstring("RHmin set to: "); uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
  }

  /* Catch new active values --> Set min. temperature */
  else if (strncmp(mp_buffer, "set.tmin:", 9) == 0){
    values.min_temp = cut_token();
    uart_sendstring("Tmin set to: "); uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
  }

  /* Catch new active values --> Set temperature deltaT */
  else if (strncmp(mp_buffer, "set.deltat:", 11) == 0){
    values.deltaT = cut_token();
    uart_sendstring("deltaT set to: "); uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
  }

  /* Catch new active values --> Set relative humidity deltaRH */
  else if (strncmp(mp_buffer, "set.deltarh:", 12) == 0){
    values.deltaRH = cut_token();
    uart_sendstring("deltaRH set to: "); uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
  }

  /* Catch new active values --> Set sensor delay
   *
   * NOTICE:
   * Using a cast here. Sorry for that, that isn't nice and doesn't
   * produce the intended effect. But, *hrm*... You know, it works ;)
   */
  else if (strncmp (mp_buffer, "set.delay:", 10) == 0){
    values.sensor_delay = (int) cut_token() + 0.5; /* CASTlevania */
    values.sensor_delay = values.sensor_delay/30; /* Because 30s/6=5 */
    if (values.sensor_delay < 1){
      values.sensor_delay = 1;
    }
    sprintf(mp_buffer, "%i", values.sensor_delay*30);
    uart_sendstring("Sensor delay set to: "); uart_sendstring(mp_buffer);
    uart_sendchar(*lf);
  }

  /* Display power on/off (NOT the backlight!) */
  else if (strncmp (mp_buffer, "set.disp.pwr:", 13) == 0){
    int val = (int) cut_token() + 0.5; /* CASTlevania */

    if (val == 1){
      powerDisplayFlag = TRUE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
      uart_sendstring("Display power on.");
      uart_sendchar(*lf);
    }
    else{
      powerDisplayFlag = FALSE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
      uart_sendstring("Display power off.");
      uart_sendchar(*lf);
    }
  }

  /* Display Backlight power on/off */
  else if (strncmp (mp_buffer, "set.disp.bklt:", 14) == 0){
    int val = (int) cut_token() + 0.5; /* CASTlevania */

    if (val == 1){
      powerDisplayBacklightFlag = TRUE;
      PORTA |= (1<<PA4);
      uart_sendstring("Display backlight power on.");
      uart_sendchar(*lf);
    }
    else{
      powerDisplayBacklightFlag = FALSE;
      PORTA &= ~(1<<PA4);
      uart_sendstring("Display backlight power off.");
      uart_sendchar(*lf);
    }
  }

  /* Invert/Deinvert display */
  else if (strncmp (mp_buffer, "set.disp.inv:", 13) == 0){
    int val = (int) cut_token() + 0.5; /* CASTlevania */

    if (val == 1){
      invertDisplayFlag = TRUE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
      uart_sendstring("Display inverted.");
      uart_sendchar(*lf);
    }
    else{
      invertDisplayFlag = FALSE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
      uart_sendstring("Display normal.");
      uart_sendchar(*lf);
    }
  }
  else {
    uart_sendstring("Possible commands:\nping\nget.data:\nget.values\nset.server\nset.tmin:\nset.deltat:\nset.deltarh:\nset.rhmin:\nset.hadiff:\nset.delay:\nset.disp.pwr:\nset.disp.inv:\nset.disp.bklt:\n");
  }
}

void check_button_pressed(uint8_t doLogging, uint8_t powerDisplayFlag,
  uint8_t invertDisplayFlag, uint8_t powerDisplayBacklightFlag){

  /* Check if the "eject" button is pressed */
  if (doLogging == TRUE && check_logging() == FALSE){
    doLogging = FALSE;
    LCD_ShowString(0, 84, RED, "Prepared to eject SD");
    LCD_ShowString(20, 69, RED, "Wait until next");
    LCD_ShowString(12, 59, RED, "interval is over!");
  }

  /* Display power on/off (NOT backlight!) */
  if (check_power_display() == TRUE){
    if (powerDisplayFlag == FALSE){
      powerDisplayFlag = TRUE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
    }
    else if (powerDisplayFlag == TRUE){
      powerDisplayFlag = FALSE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
    }
  }

  /* Invert display toggle */ //FIXME Seems this is running 2 times?!
  if (check_invert_display() == TRUE){
    if (invertDisplayFlag == FALSE){
      invertDisplayFlag = TRUE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
    }
    else if (invertDisplayFlag == TRUE){
      invertDisplayFlag = FALSE;
      LCD_init(invertDisplayFlag, powerDisplayFlag);
    }
  }

  /* Backlight display toggle */
  if (check_power_backlight_display() == TRUE){
    if (powerDisplayBacklightFlag == FALSE){
      powerDisplayBacklightFlag = TRUE;
      PORTA ^= (1<<PA4);
    }
    else if (powerDisplayBacklightFlag == TRUE){
      powerDisplayBacklightFlag = FALSE;
      PORTA ^= (1<<PA4);
    }
  }
}

/* Check conditions for running fans. */
void check_fan(double haIn, double haOut, double rhIn,
               double tempIn, double tempOut){
  if (
      ((haIn-haOut) >= values.min_ha_diff) &&
      (rhIn > values.min_rh) &&
      (tempIn > values.min_temp) &&
      (tempIn >= tempOut)
     ){
      /* Don't ignore the delta$-values :-P */
      switch (firstCycleFlag){
        case 1  :  PORTB &= ~(1<<PB3);
                   fan.running = 1;
                   firstCycleFlag = FALSE;
                   break;
        case 0  :  if (
                        (tempIn > (values.min_temp+values.deltaT)) &&
                        (rhIn > (values.min_rh+values.deltaRH))
                      ){
                        PORTB &= ~(1<<PB3);
                        fan.running = 1;
                       }
                   break;
      }
  }
  else{
    PORTB |= (1<<PB3);
    fan.running = 0;
  }
}

/* Initialize some default values, (changeable over Network/RS232) */
void default_value_init(){
  values.min_ha_diff = HADIFF;
  values.min_rh = RHMIN;
  values.min_temp = TMIN;
  values.deltaT = DELTAT;
  values.deltaRH = DELTARH;
  values.sensor_delay = DELAY; /* 540s on 160px ~= 24 hours on display */
}

void init_timer(){
  /* Dear father,
   *
   * these interrupt-timer-overflow-thingie has shorten
   * my lifespan much, i'm sure.
   * Please Atmel, more examples in your docs would
   * be very nice!
   *
   * Thanks for listening.
   *
   * lifespan--;
   */

  /* Controlregister set to CTC, prescaler 256 --> ~5s@4Mhz */
  TCCR1B |= (1<<CS12)|(0<<CS11)|(0<<CS10);
  TIMSK |= (1<<TOIE1);
}

void send_data(uint32_t lastLines){
  sd_raw_sync();

  LCD_ShowString(2, 117, LILAC, "Sending data...");
  uart_sendstring(header);

  if (!fd){
    #if DEBUG
      error_report(11);
    #endif
  }
  else{
    /* Seek current offset and save it to currentFile_pos */
    int32_t currentFile_pos = 0;

    if (!fat_seek_file(fd, &currentFile_pos, FAT_SEEK_CUR)){
      #if DEBUG
        error_report(12);
      #endif
    }

    uint8_t buffer[8];
    uint32_t offset = 0;
    intptr_t count;
    int32_t searchFile_pos = currentFile_pos - 1;

    /* Prevent fat_seek_file to return an instant 0 and break it.
     * Also prepare searchFile_pos = 0 when everything is
     * requested (get.data:0 or get.data:).
     */
    if (lastLines == 0){
      searchFile_pos = 0;
    }

    if (!fat_seek_file(fd, &searchFile_pos, FAT_SEEK_SET)){
        #if DEBUG
          error_report(15);
        #endif
    }

    /* Counting \n to send only the lines requested
     * (only if lastLines != 0)
     */
    if (lastLines != 0){
      lastLines++; /* I need this line, really. Why? Don't get it.*/

      /* How many bytes do we have to move offset back? */
      while ((fat_read_file(fd, buffer, 1) > 0) && (lastLines > 0)){
        if (buffer[0] == '\n'){
          lastLines--;
        }

        searchFile_pos = searchFile_pos - 1;

        if (!fat_seek_file(fd, &searchFile_pos, FAT_SEEK_SET)){
          #if DEBUG
            error_report(14);
          #endif
        }
      }

      /* Move offset to new position*/
      int32_t newFile_pos = searchFile_pos +1;

      if (!fat_seek_file(fd, &newFile_pos, FAT_SEEK_SET)){
        #if DEBUG
          error_report(13);
        #endif
      }
    }

    while ((count = fat_read_file(fd, buffer, sizeof(buffer))) > 0){
      for(intptr_t i = 0; i < count; ++i){
        uart_sendchar(buffer[i]);
      }
      offset += 8;
    }

    /* Send an "correct" EOF so we know when transmission is done*/
    uart_sendstring("EOTX\n");

    /* Move offset to EOF again */
    fat_seek_file(fd, &currentFile_pos, FAT_SEEK_SET);

  }

  /* Data send, stop reading, then SNAFU */
  LCD_ShowString(2, 117, LILAC, "               ");
}

/* Cut an token out of a string. Used in "Network" ISR down below. */
double cut_token(){
  char delimitChar[] = ":"; /* Delimiter */
  char *cNewValue;
  double dNewValue;

  cNewValue = strtok(mp_buffer, delimitChar);

  for ( int i=0; i<1; i++ ){
    cNewValue = strtok(NULL, delimitChar);
  }

  dNewValue = atof(cNewValue);
  sprintf(mp_buffer, "%02.1f", dNewValue);
  return dNewValue;
}

/* Is button 2 pressed to invert the display? */
uint8_t check_invert_display(){
  int buttonValue = 0;

  /* Get an average value from the button */
  for (int i = 0; i< 3; i++){
    buttonValue += adcReadOnce(BUTTONPIN);
    buttonValue = buttonValue/3;
  }

  /* Is it button No.2? */
  if ((buttonValue >= 155) && (buttonValue <= 175)){
    return TRUE;
  }
  else{
    return FALSE;
  }
}

uint8_t check_power_display(){
  int buttonValue = 0;

  /* Get an average value from the button */
  for (int i = 0; i< 3; i++){
    buttonValue += adcReadOnce(BUTTONPIN);
    buttonValue = buttonValue/3;
  }

  /* Is it button No.3? */
  if ((buttonValue >= 120) && (buttonValue <= 150)){
    return TRUE;
  }
  else{
    return FALSE;
  }
}

uint8_t check_power_backlight_display(){
  int buttonValue = 0;

  /* Get an average value from the button */
  for (int i = 0; i< 3; i++){
    buttonValue += adcReadOnce(BUTTONPIN);
    buttonValue = buttonValue/3;
  }

  /* Is it button No.4? */
  if ((buttonValue >= 95) && (buttonValue <= 115)){
    return TRUE;
  }
  else{
    return FALSE;
  }
}

/* ISR for save ejecting the SD and stuff for the display */
ISR(TIMER1_OVF_vect){
  cli();

  /* Read sensors every 30s in main(), main() decides if only text or
   * text AND graph should be updated.
   * cntr_isr is set to 0 again in main().
   * Also used to check if buttons are pressed.
   */
  cntr_isr++;

  /* Check buttons only every 5s in main().
   * buttonCheckFlag set to FALSE in main() again. */
  buttonCheckFlag = TRUE;

  sei();
}

/* ISR called on incoming data.
 *
 * NOTICE:
 * I know a ISR should be short as possible. But in this case...
 *
 *                >>> I DONT'T CARE :-) <<<
 */
ISR(USART_RXC_vect){
  cli();

  uart_getstring(uart_rcv, sizeof(uart_rcv));
  uartCheckFlag = TRUE;

  sei();
}

/* Your're done! EOF reached!       \o/       <<<< THE END >>>> */
