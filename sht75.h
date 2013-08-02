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
 */

#ifndef __sht75_h
#define __sht75_h

#define SHT_TEMPERATURE 0x03	/* Measure temp - for ShtMeasure */
#define SHT_HUMIDITY 		0x05	/* Measure humidity - for ShtMeasure */

#define SHT_DDR			DDRB	/* Port with clock and data pins */
#define SHT_PORT		PORTB	/* Port with clock and data pins */
#define SHT_PIN			PINB	/* Port with clock and data pins */
#define SHT_CLOCK		0		  /* Pin used to output clock to SHT */

/*
 * Yaihh! Wtf?! Why is the data-pin-#define commented out? :)
 *
 * #define SHT_DATA		1		// Pin used to read/output data from/to SHT
 *
 * NOTICE:
 *  ...because we want to drive more then one sht75. An #define is
 *  replaced by the preprocessor. We don't want hardcoded pins, we want
 *  to talk to MANY sht75. (ok, maximum 6 yet without changing too much)
 *  So i've modified all functions where SHT_DATA was replaced by $PIN.
 *  Now we can call 'SHTfoo(int pin)'. Clockline can be left alone, i
 *  think.
 */

#define SHT_DELAY		25		/* uS delay between clock rise/fall */

#define STATUS_REG_W 		0x06 		/* Command to read status register */
#define STATUS_REG_R 		0x07 		/* Command to write status register */
#define RESET 		0x1e /* Command for soft reset (not currently used) */

/* Struct for the Data from the Sensor */
typedef struct sht75st{
	double Temperature;
	double Humidity;
	double Dewpoint;
	double AbsolutHumidity;
} sht75_array;

void ShtInit(int datapin);
void ShtReset(int datapin);
int  ShtMeasure(unsigned char mode, int datapin);
void ShtReadEverything(sht75_array *sht75_struct, int datapin);
char ShtReadStatus(unsigned char *p_value, int datapin);
/* NOTICE: Commentet to safe some memory. Don't need it yet. */
//char ShtWriteStatus(unsigned char value, int datapin);

#endif
