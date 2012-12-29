#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <inttypes.h>
#include <util/delay.h>
#include <stdlib.h>
#include "utils.h"
#ifdef USE_UARTSYNC
  #include "uartsync.h"
#else
  #include "uart.h"
#endif
#include "term.h"

void phexch(unsigned char c)
{
  uart_putc(c + ((c < 10) ? '0' : 'A' - 10));
}
void phex(unsigned char c) { phexch(c >> 4); phexch(c & 15); }
void phex16(unsigned int i) { phex(i >> 8); phex(i); }

// Just a wrapper around uart_puts_p, requires
// the string to be wrappd with PSTR macro
void term_sendstr(const char *s)
{
  uart_puts_p(s);
}

// Receive a string from the UART serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
// The return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
int16_t term_recvstr(char *buf, uint8_t size)
{
  unsigned int ur;
  unsigned char r;
  uint8_t count = 0;
  unsigned int i = 0;

#ifdef USE_UARTSYNC
  if(! uart_char_is_waiting())
    return 0;
#endif

  // blocks until new line is received or size is exhausted
  while (count < size && i < 200)
  {
#ifdef USE_UARTSYNC
    r = uart_getc();
#else
    ur = uart_getc();

    // return immediately since no data to begin with
    if((ur & UART_NO_DATA) && count==0)
      return 0;

    // nothing to process
    if(ur & UART_NO_DATA)
      continue; 

    // some errors
    if((ur & UART_FRAME_ERROR) ||
       (ur & UART_PARITY_ERROR) ||
       (ur & UART_OVERRUN_ERROR) )
    {
      return -1;
    }

    r = (unsigned char) ur;
#endif

    if(r == '\r' || r == '\n' || r == '\0')
    {
      *buf = '\0'; // last one is null terminated
      return count;
    }
    if (r >= ' ' && r <= '~')
    {
      *buf++ = r;
      count++;
    }

    // we are reading too slow, continue
    // if(ur & UART_BUFFER_OVERFLOW) continue;
  } 

#ifdef USE_UARTSYNC

#else
  uart_flush();
#endif

  *buf = '\0'; // null terminated
  return count;
}


uint8_t term_check_mode(const char *s, uint8_t n)
{
  if(n<2)
    return MODE_INVALID;

  if(s[0]==':' && s[1]=='d') {
    // term_sendstr(PSTR("draw mode\r\n"));
    return MODE_DRAW;
  }
  if(s[0]==':' && s[1]=='t') {
    // term_sendstr(PSTR("timeset mode\r\n"));
    return MODE_TIMESET;
  }
  return MODE_INVALID;
}


// each line represents a command
// and each of these commands should have a handler
// that knows how to draw things


