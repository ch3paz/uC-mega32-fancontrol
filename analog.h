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

uint16_t adcInit(void);
uint16_t adcReadOnce(uint8_t channel);
