#ifndef UARTSYNC_H
#define UARTSYNC_H

#include <inttypes.h>
#include <stdio.h>

uint8_t uart_char_is_waiting(void);
void uart_putchar(char c, FILE *stream);
char uart_getchar(FILE *stream);
void uart_putc(char c);
char uart_getc(void);

void uart_init(void);

void uart_puts(const char *s );
extern void uart_puts_p(const char *s );
#define uart_puts_P(__s)       uart_puts_p(PSTR(__s))

/* http://www.ermicro.com/blog/?p=325 */

// FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
// FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

#endif
