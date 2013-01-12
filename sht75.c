/*
 * Sensirion SHTxx Humidity Sensor Library
 *
 * Library for using the SHT1x humidity and temperature
 * sensors from Sensirion (http://www.sensirion.com).
 * Based on Sensirion application note "Sample Code SHTxx",
 * adapted by Brian Low (mportobello@hotmail.com)
 *
 * History:
 * 2003-Jul-03	BL   - Created
 * 2005-Mar-05  bue - Adapted by Daniel Buergin
 * 2011-Jun-05  che - Adapted by Joeran Zeller
 *
 * Knowhow:
 *     - Making a Pin high: PORTB |= (1<<2);
 *     - Making a Pin low : PORTB &= ~(1<<2);
 */

#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include "sht75.h"

/* Set the DDR Register to 1 (high). Result: PIN is ready for Output */
void enable_data(int datapin) 	{ SHT_DDR |= (1 << datapin); }

/*
* Set the DDR Register to 0 (low) and the PORT Register to 1 (high)
* Result: PIN is ready for Input with Pull-UP
*/
void disable_data(int datapin)	{ SHT_DDR  &= ~(1 << datapin);
			  SHT_PORT |= (1 << datapin); }

/* Set DATA PORT Register to 1 (high) */
void data_high(int datapin)	{ SHT_PORT |= _BV(datapin); }

/* Set DATA PORT Register to 0 (low) */
void data_low(int datapin)	{ SHT_PORT &= ~_BV(datapin); }

/* Set CLOCK PORT Register to 1 (high) */
void clock_high(void)	{ SHT_PORT |= _BV(SHT_CLOCK); }

/* Set CLOCK PORT Register to 0 (low) */
void clock_low(void)	{ SHT_PORT &= ~_BV(SHT_CLOCK); }

/* Set CLOCK DDR Register to 1 (high) */
void enable_clock(void)	{ SHT_DDR |= (1 << SHT_CLOCK); }

/* Initialize AVR i/o pins. */
void ShtInit(int datapin){
	enable_clock();
	clock_low();
	disable_data(datapin);
}

/*
* Generates a transmission start
*       _____         ________
* DATA:      |_______|
*           ___     ___
* SCK : ___|   |___|   |______
*
*/
void transstart(int datapin){
	enable_data(datapin);
	asm volatile ("nop"::);
	data_high(datapin);
	clock_low();
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	clock_high();
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	data_low(datapin);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	clock_low();
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	clock_high();
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	data_high(datapin);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	clock_low();
}

/*
* Communication reset: DATA-line=1 and at least 9 SCK cycles
* followed by transstart
*       _____________________________________________________         ________
* DATA:                                                      |_______|
*          _    _    _    _    _    _    _    _    _        ___     ___
* SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
*/
void ShtReset(int datapin){
	unsigned char i;

	/* Set DATA Line to Input */
	enable_data(datapin);
	asm volatile ("nop"::);
	data_high(datapin);

	/* Prepare CLOCK-Line for Pulses */
	clock_low();

	/* Send 9 Pulses on the CLOCK-Line */
	for(i=0;i<10;i++){
		clock_high();
		asm volatile ("nop"::);
		asm volatile ("nop"::);
		clock_low();
		asm volatile ("nop"::);
		asm volatile ("nop"::);
	}
	transstart(datapin); /* Transmission start */
}

/*
* Reads a byte form the Sensibus and gives an acknowledge
* in case of "ack=1"
*/
char read_byte(int datapin){
	unsigned char i,val=0;

	 /*
	 * Set the DATA Line to Input.
	 * Important: If there is no external Pull-Up Resistor
	 */

	disable_data(datapin);
	asm volatile ("nop"::);

	/* Read in 8 bits, LSB first */
	for (i=0x80;i>0;i/=2){
		clock_high();
		asm volatile ("nop"::);
		asm volatile ("nop"::);

		if (bit_is_set(SHT_PIN, datapin)){
			val=(val | i); // Got a Bit
		}

		clock_low();
		asm volatile ("nop"::);
		asm volatile ("nop"::);
	}

	/* Send ACK */
	enable_data(datapin);
	asm volatile ("nop"::);
	data_low(datapin);
	asm volatile ("nop"::);
	clock_high();
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	clock_low();
	asm volatile ("nop"::);
	asm volatile ("nop"::);
	disable_data(datapin);

	return val;
}

/*
* Writes a byte on the Sensibus and checks the acknowledge.
* Returns 0 if the successful
*/
char write_byte(unsigned char value, int datapin){
	unsigned char i;
	unsigned char error = 0;

	/* Set the DATA Line to Output */

	enable_data(datapin);
	asm volatile ("nop"::);

	/* Write each bit one at a time, LSB first */

	for (i=0x80;i>0;i/=2){
		if (i & value){
			data_high(datapin); /* Write a 1 */
		}
		else{
			data_low(datapin);  /* Write a 0 */
		}

		clock_high();
		asm volatile ("nop"::);
		asm volatile ("nop"::);
		clock_low();
		asm volatile ("nop"::);
		asm volatile ("nop"::);
	}

	disable_data(datapin);
	asm volatile ("nop"::);

	/* Read ACK */
	clock_high();
	asm volatile ("nop"::);
	asm volatile ("nop"::);

	error = bit_is_set(SHT_PIN, datapin);
	clock_low();
	return error; /* error=1 in case of no acknowledge */
}

/*
* Read temperature from the sensor.
* Returns 0xFFFF if the measurment failed
*/
int ShtMeasure(unsigned char mode, int datapin){
	unsigned int 	temp = 0xFFFF;
	/* unsigned char	checksum; */ /* Don't see the need of this yet */
	unsigned char 	c;
	unsigned int    error = 0;

	/* Signal start of communications */
	transstart(datapin);

	/* Request measurement */
	error = write_byte(mode, datapin);
	if (error > 0){
		/* printf("Error in write_byte\n"); */ /* Why's/What's that? */
	}

	/*
	* Sensor lowers the data line when measurement
  * is complete. Wait up to 2 seconds for this.
	*/
	for (c=0; c<20; c++){
		if (! bit_is_set(SHT_PIN, datapin)){
			break;
		}
		_delay_ms(300);
	}

	/* Read the measurement */
	if (! bit_is_set(SHT_PIN, datapin)){
		temp = read_byte(datapin);
		temp = temp << 8;
		temp += read_byte(datapin);
		/* checksum = read_byte(); */
	}

	return temp;
}

/*
* Read the Sensordata Temperature and Humidity and calculates
* the real Temperature and the Temperature compensated Humidity
* Calculates also the DewPoint
*
* input: Pointer to a Struct
* ouput: Nothing
*/
void ShtReadEverything(sht75_array *sht75_struct, int datapin){
	int T;
	int RH;
	double Habsolut = 0.0;
	double RHlinear = 0.0;
	double RHtrue   = 0.0;
	double Temp     = 0.0;
	double logEx    = 0.0;
	double DewPoint = 0.0;

	T   = ShtMeasure(SHT_TEMPERATURE, datapin);
	RH  = ShtMeasure(SHT_HUMIDITY, datapin);

	/* Calculate True Temperature (see also Sensirion SHT75 Datasheet) */
	Temp     = ((double) T)*0.01 - 40.0;

	/* Calculate True Humidity (see also Sensirion SHT75 Datasheet)
	 *
	 * NOTICE:
	 *  Sht75-datasheet seems to be different in this point (Sensor
	 *  version differs V3/V4). I don't know if the changed values are
	 *  really relevant. Taken the values from my V4 sht75 now.
	 *
	 *  Original term was:
	 *  RHlinear = -4.0 + 0.0405 * ((double) RH) - 0.0000028 * ((double) RH * (double) RH);
	 */
	RHlinear = -2.0468 + 0.0367 * ((double) RH) - 0.0000015955 * ((double) RH * (double) RH);
	RHtrue   = (Temp - 25) * (0.01 + 0.00008 * (double) RH) + RHlinear;

	/* Calculate Dewpoint (see also Sensirion SHT75 Datasheet)
	 *
	 * NOTICE:
	 *  Another term that i can't equate to my sht-75-datasheet. Maybe i'm
	 *  to dump for math. Again taken the term from my V4 sht75-datasheet
	 *  to calculate dewpoint.
	 *
	 *  Original term was:
	 *  logEx    = 0.66077 + (7.5 * Temp / (237.3 + Temp)) + (log10 (RHtrue) -2);
	 *  DewPoint = ((0.66077 - logEx) * 237.3) / (logEx - 8.16077);
	 *
	 *	First version:
	 *  Now we go on with a big () mess that doesn't really show up any
	 *  differences to the term above... :)
	 */
	/*DewPoint = (
	            243.12 * (((log ((double) RHlinear / 100)) +
	            ((17.62 * Temp) / (243.12 + Temp))) / (17.62 -
	            (log ((double) RHlinear / 100)) -
	            ((17.62 * Temp) / (243.12 + Temp))))
	           );
	*/

	/* Second version:
	 * Codesample taken from Sensirion "Humidity at a Glance" PDF
	 * If this isn't the truth - what then?
	 */
	logEx = (log10(RHtrue)-2.0)/0.4343+(17.62*Temp)/(243.12+Temp);
	DewPoint = 243.12*logEx/(17.62-logEx);

	/* ABSOLUTE humidity - Code grabbed also from PDF above */
	Habsolut = 216.7*(RHtrue/100.0*6.112*exp(17.62*Temp/(243.12+Temp))
						 /(273.15+Temp));

	(*sht75_struct).Temperature = Temp;
	(*sht75_struct).Humidity    = RHtrue;
	(*sht75_struct).Dewpoint    = DewPoint;
	(*sht75_struct).AbsolutHumidity = Habsolut;
}

/* Reads the status register with checksum (8-bit) */
char ShtReadStatus(unsigned char *p_value, int datapin){
	unsigned char error=0;
	/* unsigned char checksum=0; */ /* Don't see the need of this yet */

	transstart(datapin); 			/* Transmission start */
	error = write_byte(STATUS_REG_R, datapin); 	/* Send command to sensor */
	*p_value = read_byte(datapin); 	/* Read status register (8-bit) */
	/* checksum = read_byte(); */	/* Read checksum (8-bit) */
	return error; 	/* error=1 in case of no response form the sensor */
}

/*
* Writes the status register . Note this library only supports the default
* 14 bit temp and 12 bit humidity readings.
*/

/* Commentet to safe some memory. Don't need it yet.
 */

//char ShtWriteStatus(unsigned char value, int datapin){
//unsigned char error=0;
//transstart(datapin); 			/* Transmission start */
//error += write_byte(STATUS_REG_W, datapin);	/* Send command to sensor */
//error += write_byte(value, datapin); 	/* Send value of status register */
//return error; 	/* error>=1 in case of no response form the sensor */
//}
