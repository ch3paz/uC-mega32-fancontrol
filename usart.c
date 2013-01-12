/*
 * Simple, and possible a bit crappy, functions to make working with
 * USART a bit less painfull.
 *
 * Code ripped, adapted and missused from different sources by
 * Joeran Zeller ;)
 *
 * NOTICE:
 *   Automagic compute UNRR_VALUE for UBRRL in init_USART().
 *   F_CPU has to be defined for this to work somewhere!
 */

#define BAUD 230400

#include <avr/io.h>
#include <util/setbaud.h>

void uart_init(void){
  /* UART enable RXTX and Interrupt, don't forget sei() in main() */
  UCSRB |= (1<<TXEN) | (1<<RXEN) | (1<<RXCIE);
  UCSRC |= (1<<URSEL) | (3<<UCSZ0); /* Mode: Asynchron 8N1 */
  UBRRH = 0; /* Highbyte is 0 */
 /*
  * UBRR_VALUE calculated automagic through <util/setbaud.h>.
  * Else: UBRRL = (Frequency_in_Hz / (Baudrate * 16) - 1)
  */
  UBRRL = UBRR_VALUE;
}

void uart_sendchar(unsigned char c){
  while(!(UCSRA & (1<<UDRE))){ /* Wait until sending is possible */
  }
  UDR = c; /* Write to output */
}

void uart_sendstring(char *s){
  while(*s){
    uart_sendchar(*s);
    s++;
  }
}

uint8_t uart_getchar(void){
  while (!(UCSRA & (1<<RXC))){ /* Wait until char is available */
    ;
  }
  return UDR; /* Return received char */
}

void uart_getstring( char* Buffer, int MaxLen ){
  int NextChar;
  int StringLen = 0;
  NextChar = uart_getchar(); /* Wait and get next received char */

  while( NextChar != '\n' && StringLen < MaxLen - 1 ){
    *Buffer++ = NextChar;
    StringLen++;
    NextChar = uart_getchar();
  }

  *Buffer = '\0'; /* Terminate string correct */
}
