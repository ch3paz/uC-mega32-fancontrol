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

void uart_init(void);
void uart_sendchar(unsigned char c);
void uart_sendstring(char *s);
void uart_getstring( char* Buffer, int MaxLen );
int uart_getchar(void);
