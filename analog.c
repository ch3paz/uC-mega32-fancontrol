/*
* Functions to initialize the ADC and read from it.
* Code partial ripped and adapted from www.mikrocontroller.net
*
* Copyright (c) 2010-2011 by Joeran Zeller <mx-bounce@gmx.de>
*
* This file is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <avr/io.h>

/* ADC init */
uint16_t adcInit(void){
  uint16_t result;

  ADMUX = (0<<REFS1) | (1<<REFS0);  // AVcc as reference
//  ADMUX = (1<<REFS1) | (1<<REFS0);  // internal reference voltage
  ADCSRA = (1<<ADPS1) | (1<<ADPS0); // prescaler
  ADCSRA |= (1<<ADEN);              // activate ADC

  /* Dummyread of ADC */
  ADCSRA |= (1<<ADSC);          // one ADC-conversion
  while (ADCSRA & (1<<ADSC) ){} // wait until conversion is done
  result = ADCW;                // read ADCW once

  return result;
}

/* ADC read once */
uint16_t adcReadOnce( uint8_t channel ){
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F); // Select channel
  ADCSRA |= (1<<ADSC);           // one "single conversion"
  while (ADCSRA & (1<<ADSC) ){}  // wait until conversion is done

  return ADCW;                   // return ADC value
}
