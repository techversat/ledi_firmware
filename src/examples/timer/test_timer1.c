#ifndef F_CPU
#define F_CPU 8000000
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <inttypes.h>
#include <util/delay.h>
#include "calibration.h"
#include "utils.h"
#include "uart.h"


/*
Refer to atmega328p pdf pg. 94 (8-bit Timer/Counter0 with PWM)
  Main clock is 8MHz, so we need biggest prescaler

  Timer0 can be clocked internally or by external clock source on T0 pin.
  
  TCNT0       counter
  TCCR0A      Timer/Counter Control Register A
  TCCR0B      Timer/Counter Control Register B
  OCR0A       output compare register (compared with TCNT0 at all times)
  OCR0B       output compare register (compared with TCNT0 at all times)
    OC0A      output pins for compare match event
    OC0B      output pins for compare match event
    OCF0A     flag set on compare match event
    OCF0B     flag set on compare match event
  TIFR0    timer interrupt flag register
  TIMSK0   timer interrupt mask register

  The clock source is selected by CS0,CS1,CS2 bits on TCCR0B

  [TCCR0A] pg.106
  COM0A1  COM0A0  COM0B1 COM0B0 ----  ----  WGM01 WGM00
  * mainly used to generate output on OC0x pins
  * WGM01 = 1, WGM00 = 0  clears timer on OCR0A match
  * WGM01 = 0, WGM00 = 0  normal counter
  * third waveform generator register WGM02 is located on TCCR0B

  [TCCR0B] pg.109
  FOC0A   FOC0B  ----  ----  WGM02  CS02  CS01  CS00
  * FOC0x are the force output compare registers 
  * WGM02 corresponds to the waveform generator registers
  * CS02 = 1, CS01 = 0, CS00 = 1 1024 prescaler

  [TIMSK0] pg.111
  ----  ----  ----  ----  ----  OCIE0B  OCIE0A  TOIE0
  * OCIE0B timer/couter output comparet match B interrupt enable
  * OCIE0A timer/couter output comparet match A interrupt enable
  * TOIE0  timer/couter overflow interrupt enable

  [TIFR0] pg.111
  ----  ----  ----  ----  ----  OCF0B  OCF0A  TOV0
  * TOV0 timer overflow flag is set when overflow occurs
    It is cleared by hardware when executing the interrupt handling vector.
*/
#define USER_TIMER0_OVERFLOW
void timer0_init()
{
  // prescaler = 1024, overflow counter = 255 gives rise to 32Hz interrupts
  // (1<<23)/(1<<10)/ 0xff = (8388608/1024/255) = 32
#ifdef USE_TIMER0_OVERFLOW
  TCCR0A = 0x00;                  // normal timer
  TCCR0B = (1<<CS02) | (1<<CS00); // prescale by 1024, gives (1<<23)/(1<<10) = 8192 ticks
  TIMSK0 = (1<<TOIE0);            // overflow interrupt enable
  TIFR0  = (1<<TOV0);             // clear the interrupt flag, hardware clears it everytime we hit the interrupt
#else
  TCCR0A = (1<<WGM01);            // clear timer on compare match (CTC)
  TCCR0B = (1<<CS02) | (1<<CS00); // prescale by 1024, gives (1<<23)/(1<<10) = 8192 ticks
  TIMSK0 = (1<<OCIE0A);            // overflow interrupt enable
  TIFR0  = (1<<OCF0A);             // clear the interrupt flag, hardware clears it everytime we hit the interrupt
  OCR0A  = 255;
#endif

}

#ifdef USE_TIMER0_OVERFLOW
ISR(TIMER0_OVF_vect)
{
  PORTC ^= (1<<PC4);
}
#else
ISR(TIMER0_COMPA_vect)
{
  PORTC ^= (1<<PC4);
}
#endif

void rtc_init(void)
{
  TCCR2A = (1<<WGM21);  // 0x02;//clear timer on compare match (CTC)
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // pg.162 prescale by 1024, gives 32 ticks
  TIMSK2 = (1<<OCIE2A); // 0x02; //enable timer2A output compare match interrupt
  TIFR2  = (1<<OCF2A); //clear the interrupt flag
  OCR2A  = 31;   //Set the output compare to 31 since the count starts at 0 - this gives 32 interrups per sec
  ASSR   = (1<<AS2); //pg.164 enable asynchronous mode, disable this line if you are using the 32khz crystal as a the system clock source
}


ISR(TIMER2_COMPA_vect)
{
  // toggle pin
  PORTC ^= (1<<PC5);
}


int main()
{
  uint8_t i = 0;

  do_calibration();

  rtc_init();
  timer0_init();

  uart_init( UART_BAUD_SELECT(38400,F_CPU) );
  uart_putc(OSCCAL);  
  uart_putc('\n'); 

  DDRC |= (1<<PC4) | (1<<PC5);  // both pins output

  sei(); // turn on interrupt handler

  while(1)
  {
    delay_ms(100);
    // while ((ASSR & (1<<OCR2BUB)) != 0) {} 
  }
}
