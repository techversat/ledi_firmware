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
#include <stdlib.h>
#include "calibration.h"
#include "utils.h"
#include "ht1632.h"
#ifdef USE_UARTSYNC
  #include "uartsync.h"
#else
  #include "uart.h"
#endif
#include "term.h"

// #define USE_V0

#define BUTTON1 PC1
#define BUTTON2 PC0

#ifdef USE_V0
  #define BUZZER_PORT PORTB
  #define BUZZER_REG  DDRB
  #define BUZZER      PB1
#else
  #define BUZZER_PORT PORTD
  #define BUZZER_REG  DDRD
  #define BUZZER      PD6
#endif

#define BT_PRESS 0x02
#define BT_HOLD  0x04

// define some notes
// Frequencies from http://www.phy.mtu.edu/~suits/notefreqs.html
// converted to half-periods (us) by calculating
//  1000000/2/frequency
// where frequency is in Hz
#define D5 851
#define E5 758
#define Fsh5 675
#define G5 637
#define A5 568
#define B5 506
#define C6 477
#define D6 425
#define DUR 40


volatile uint8_t seconds,minutes,hours;
volatile uint8_t days,months,years,leap_year;
volatile unsigned char flag = 0;
volatile uint8_t last_buttonstate = 0;
volatile uint8_t last_button1state = 0;
volatile int intcount = 0;

#define BUFSIZE 255
volatile uint8_t MODE = 0;
static char BUF[BUFSIZE];  // 2 panels = 128bytes, 12 bytes overhead for each panel with some enough wiggle room
static uint8_t BUFI;


void update_datetime(uint8_t);


void start_led_sync_timer()
{
  // prescaler = 1024, overflow counter = 255 gives rise to 32Hz interrupts
  // (1<<23)/(1<<10)/ 0xff = (8388608/1024/255) = 32
  // TCCR0A = 0x00;                  // normal timer
  // TCCR0B = (1<<CS02) | (1<<CS00); // prescale by 1024, gives (1<<23)/(1<<10) = 8192 ticks
  // TIMSK0 = (1<<TOIE0);            // overflow interrupt enable
  // TIFR0  = (1<<TOV0);             // clear the interrupt flag, hardware clears it everytime we hit the interrupt
  TCCR0A = (1<<WGM01);            // clear timer on compare match (CTC)
  TCCR0B = (1<<CS02) | (1<<CS00); // prescale by 1024, gives (1<<23)/(1<<10) = 8192 ticks
  TIMSK0 = (1<<OCIE0A);           // overflow interrupt enable
  TIFR0  = (1<<OCF0A);            // clear the interrupt flag, hardware clears it everytime we hit the interrupt
  OCR0A  = 255;                   // would be same number as overflow
}

void stop_led_sync_timer()
{
  TCCR0B = 0x00;  // turn off the timer
}

// ISR(TIMER0_OVF_vect)
ISR(TIMER0_COMPA_vect)
{
  sei();  // enable so we become interruptible
  PORTC ^= (1<<PC4);
  ht1632_syncram_fast(); 
}

void rtc_init(void)
{
  TCCR2A = (1<<WGM21);  // 0x02;//clear timer on compare match (CTC)
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // pg.162 prescale by 1024, gives 32 ticks
  TIMSK2 = (1<<OCIE2A); // 0x02; //enable timer2A output compare match interrupt
  TIFR2  = (1<<OCF2A); //clear the interrupt flag
  OCR2A  = 31;   //Set the output compare to 31 since the count starts at 0 - this gives 32 interrups per sec
  ASSR   = (1<<AS2); //pg.164 enable asynchronous mode, disable this line if you are using the 32khz crystal as a the system clock source
}


void key_init()
{
  //Enable PIN Change Interrupt 1 (PCIE1) - This enables interrupts on pins
  //  //PCINT14...8 see p70 of datasheet
  PCICR |= (1<<PCIE1);

  //Set the mask on Pin change interrupt 1 so that only PCINT12 (PC4) triggers
  //the interrupt. see p71 of datasheet
  // PCINT17 (PD1) for hours, PCINT19 (PD3) for minutes
  // PC1 / ADC1 / PCINT9
  // PC0 / ADC0 / PCINT8
  PCMSK1 |= (1<<PCINT9) | (1<<PCINT8);

  DDRC  &= ~(1<<BUTTON1);
  PORTC |=  (1<<BUTTON1);    // tied high when button is NOT pressed
  DDRC  &= ~(1<<BUTTON2);
  PORTC |=  (1<<BUTTON2);    // tied high when button is NOT pressed
}

void extra_init(void)
{
  BUZZER_REG  |= (1<<BUZZER);
}

void play_tone(uint16_t delay, uint8_t duration)
{
  // delay is half-period in microseconds
  // duration is in 10ms increments
  
  // example: 440Hz --> delay=1136
  
  // duration = 2*delay * cycles (all in same units)
  // cycles = 10000 * duration / delay / 2
  // cycles = 100 * duration / (delay/50)
  uint16_t tmp = 100 * duration;
  uint16_t delaysm = delay / 50;
  uint16_t cycles = tmp / delaysm;
  
  while(cycles > 0) {
    BUZZER_PORT |= (1<<BUZZER);
    delay_us(delay);
    BUZZER_PORT &= ~(1<<BUZZER);
    delay_us(delay);
    cycles--;
  }
}


ISR(TIMER2_COMPA_vect)
{
  update_datetime(0);
}


ISR(PCINT1_vect)
{
  // if the button went low, the button was pressed
  if(!(PINC & (1<<BUTTON2)))
  {
    intcount++;

    // was not pressed before
    if(! (last_buttonstate & BT_PRESS))
    { 
      _delay_ms(3);                  // debounce
      if( PINC & (1<<BUTTON2) )      // filter out bounces
        return;

      last_buttonstate |= BT_PRESS;

      minutes++;
      update_datetime(1);
      play_tone(D5, 2);

      // check if it's still tied to ground
      while(!(PINC & (1<<BUTTON2)))
      {
        if(++intcount > 0xAA00)
        {
          last_buttonstate |= BT_HOLD;
          play_tone(A5, 2);
          return;
        }
      }
    }
  }
  else if(!(PINC & (1<<BUTTON1)))
  {
    _delay_ms(3);                  // debounce
    if( PINC & (1<<BUTTON1) )      // filter out bounces
      return;

    last_button1state = (last_button1state == 0); // toggle
  }
  else
  {
    intcount = 0;
    last_buttonstate &= ~(BT_PRESS | BT_HOLD);
  }
}


// SIGNAL(SIG_OUTPUT_COMPARE0A)
// ISR(TIMER2_COMPA_vect)
void update_datetime(uint8_t noincrement)
{
  if(!noincrement)
    seconds++;

  if(seconds >= 60)
  {
    seconds = 0;
    minutes++;
  }
  if(minutes >= 60)
  {
    minutes = 0;
    hours++;
  }
  if(hours >= 24)
  {
    hours = 0;
    days++;
    if( (days == 32) ||
      (days == 31 && months == (4|6|9|11)) ||
      (days == 30 && months == 2 && leap_year != 0) ||
      (days == 29 && months == 2 && leap_year == 0) )
    {
      days = 1;
      months++;
      if(months == 13)
      {
        months = 1;
        years++;
        if( (years % 400 == 0) ||
          (years % 4 == 0 && years % 100 != 0) )
        {
          leap_year = 1;
        } else {
          leap_year = 0;
        }
      }
    }
  }
}

void set_time(char* str)
{
  str[0] = (hours / 10) + '0';
  str[1] = (hours % 10) + '0';
  str[2] = (minutes / 10) + '0';
  str[3] = (minutes % 10) + '0';
  str[4] = (seconds / 10) + '0';
  str[5] = (seconds % 10) + '0';
  str[6] = 0;
}

void disp_bindots(void)
{
  uint8_t ytop = 1;
  // let's do binary clock thing here
  ht1632_sendbyte(28, ytop,   hours>>4,   1);    // upper word
  ht1632_sendbyte(28, ytop+4, hours<<0,   1);    // lower word
  ht1632_sendbyte(29, ytop,   minutes>>4, 1);    // upper word
  ht1632_sendbyte(29, ytop+4, minutes<<0, 1);    // lower word
  ht1632_sendbyte(30, ytop,   seconds>>4, 1);    // upper word
  ht1632_sendbyte(30, ytop+4, seconds<<0, 1);    // lower word
}

void disp_time(char *timebuf)
{
  static int8_t ppos = 0;
  int8_t pos = 0, ytop=1;
  ht1632_clear();
  ht1632_putchar(0,ytop,timebuf[0]);
  ht1632_putchar(6,ytop,timebuf[1]);
  ht1632_plot(12,ytop+2,1,0);
  ht1632_plot(12,ytop+4,1,0);
  ht1632_putchar(14,ytop,timebuf[2]);
  ht1632_putchar(20,ytop,timebuf[3]);
  ppos = pos;
}

void disp_time_seconds(char *timebuf)
{
  static int8_t ppos = 0;
  int8_t pos = 0;
  ht1632_clear();

  // the last arg is 'ramonly' which updates just the shadow_ram
  // Once all the digits are written in ram, we then sync the
  // entire thing in one shot - should be faster.
  ht1632_putchar(0,0,timebuf[0]);
  ht1632_putchar(5,0,timebuf[1]);
  ht1632_putchar(11,0,timebuf[2]);
  ht1632_putchar(16,0,timebuf[3]);
  ht1632_putchar(22,0,timebuf[4]);
  ht1632_putchar(27,0,timebuf[5]);

  pos = seconds%32; 
  ht1632_plotram(pos, 7,1,0);
  ht1632_plotram(ppos,7,0,0);

  ht1632_syncram();
  ppos = pos;
}

void demo_life()
{
  uint8_t x,y, neighbors, newval;
  int i;
  byte2 xmax = ht1632_xmax();
  byte2 ymax = ht1632_ymax();

  ht1632_clear();

  ht1632_plot(15,9,1,0);  // Plant an "acorn"; a simple pattern that
  ht1632_plot(17,10,1,0); //  grows for quite a while..
  ht1632_plot(14,11,1,0);
  ht1632_plot(15,11,1,0);
  ht1632_plot(18,11,1,0);
  ht1632_plot(19,11,1,0);
  ht1632_plot(20,11,1,0);
  ht1632_plot(seconds%20,10,1,0); // pseudorandom seed
  ht1632_plot(seconds%17,9,1,0); // pseudorandom seed

  if(N_PANELS>1)
  {
    ht1632_plot(seconds%33,11,1,0); 
    ht1632_plot(seconds/2,12,1,0); 
    ht1632_plot((10+seconds)%36,12,1,0); 
    ht1632_plot(34,11,1,0);
    ht1632_plot(35,12,1,0);
    ht1632_plot(36,13,1,0);
    ht1632_plot(37,13,1,0);
    ht1632_plot(38,12,1,0);
    ht1632_plot(39,12,1,0);
    ht1632_plot(40,11,1,0);
    ht1632_plot(seconds%37,13,1,0); 
  }

  _delay_ms(800);   // Play life
  ht1632_snapram();

  // 1000 iterations
  for (i=0; i < 100; i++) {
    for (x=1; x < xmax; x++) {
      for (y=1; y < ymax; y++) {
        neighbors = ht1632_getram(x, y+1) +
          ht1632_getram(x, y-1) +
          ht1632_getram(x+1, y) +
          ht1632_getram(x+1, y+1) +
          ht1632_getram(x+1, y-1) +
          ht1632_getram(x-1, y) +
          ht1632_getram(x-1, y+1) +
          ht1632_getram(x-1, y-1);

        switch (neighbors) {
        case 0:
        case 1:
          newval = 0;   // death by loneliness
          break;
        case 2:         // remains the same
          newval = ht1632_getram(x,y);
          break;  
        case 3:
          newval = 1;   // grow when you have 3 neighbors
          break;
        default:
          newval = 0;   // death by overcrowding
          break;
        }

        ht1632_plot(x,y, newval,0);
      }
    }
    if(last_button1state==0) 
    {
      ht1632_clear();
      return;
    }
    ht1632_snapram();
    _delay_ms(20);
  }
  _delay_ms(1000);
}


void mode_draw()
{
  int16_t n;
  uint8_t rc,val;
  uint8_t state = last_button1state; // button push exits this loop
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t x_mask = ht1632_xmax() - 1;
  uint8_t y_mask = ht1632_ymax() - 1;
  char *ptr;
  char save;

  ht1632_clear();

  // we start the timer when we enter this mode
  start_led_sync_timer();

  // treats each line as a command
  // we only modify the output ram - the timer takes care of the rest
  while(1)
  {
    if(last_button1state !=  state)
      return;
    
    ptr = &BUF[0];
    n = term_recvstr(ptr, BUFSIZE);

    // no data
    if(n == 0) continue;
    // error occured
    if (n < 0) break;

    if(ptr[0]=='q') break;
    if(ptr[0]=='w') 
    {
      ht1632_clearram();
      continue;
    }
    if(ptr[0]=='p')
    {
      // format: p01101 
      //         pxxyy1
      if(n > 5) 
      {
        ptr++; // skip p
        save = *(ptr+2);  
        *(ptr+2) = '\0';
        x = atoi(ptr);

        ptr+=2;
        *ptr = save; 
        save = *(ptr+2);
        *(ptr+2) = '\0';
        y = atoi(ptr);

        ht1632_plotram(x, y, (save=='1')?1:0, 1);
        // format was: p 01 10 1
        // waste of 3 bytes over wireless
        /*
        ptr+=2; *(ptr+2) = '\0';
        x = atoi(ptr);
        ptr+=3; *(ptr+2) = '\0';
        y = atoi(ptr);
        ptr+=3; *(ptr+1) = '\0';
        val = atoi(ptr);
        ht1632_plotram(x, y, val, 1);
        */
      }
      continue;
    }

    ht1632_editram_char(0, 0, ptr, n, 0);

    state = last_button1state;
  }

  stop_led_sync_timer();
}


void mode_timeset(char *strbuf, int i)
{
  uint8_t j, timegood = 0;
  uint8_t digits[4] = {0,0,0,0};
  char tok;

  play_tone(A5, 10); _delay_ms(10);
  play_tone(B5, 10); _delay_ms(10);

  timegood = 1; 
  for(j=0; j<4; j++)
  {
    tok = strbuf[j+2];
    if(tok >= '0' && tok <= '9')
    {
      digits[j] = tok - '0';      
    }
    else
    {
      timegood = 0;
    }
  }

  if(timegood)
  {
    // indicate that timeset is good
    play_tone(B5, 5); _delay_ms(2);
    play_tone(D6, 5);
    hours   = 10*digits[0] + digits[1];
    minutes = 10*digits[2] + digits[3];
  }
  else
  {
    // indicate that it's bad
    play_tone(G5, 20); _delay_ms(5);
    play_tone(G5, 20);
  }
}


void mode_msg(char *strbuf, int i)
{
  play_tone(A5, 4); _delay_ms(5);
  // play_tone(A5, 4); _delay_ms(5);
  // play_tone(B5, 5); _delay_ms(5);
  // play_tone(E5, 8);
  ht1632_scrollstr(strbuf, i, 40);
}


void sleep_and_wake()
{
  sleep_enable();    // set the sleep enable bit in the SMCR register
  sleep_cpu();       // use the sleep command
  sleep_disable();  // unset th esleep enable bit, for safety
  OCR2B = flag;
}


int main()
{
  uint8_t i = 0;
  uint8_t n = 0;
  uint8_t pos;
  char strbuf[64];
  char timebuf[16];
  uint8_t seconds_last = seconds;
  uint8_t minutes_last = minutes;
  unsigned int ch;

  // FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
  // stdin = stdout = &uart_stream;

  // always calibrate the internal RC oscillator using external 32Khz crystal
  do_calibration();
  // OSCCAL = 55;

  rtc_init();
  key_init();
  extra_init();
  ht1632_setup(1); // use ram

#ifdef USE_UARTSYNC
  uart_init();
#else

  #ifdef USE_V0
  uart_init( UART_BAUD_SELECT(9600,F_CPU) );
  #else
  // uart_init( UART_BAUD_SELECT(38400,F_CPU) );
  uart_init( UART_BAUD_SELECT(9600,F_CPU) );
  #endif

#endif

  sei(); // turn on interrupt handler

  // strncpy(strbuf, "ready: ", 7);
  // sprintf(&strbuf[7], "%d", OSCCAL);
  // ht1632_scrollstr(strbuf, 10, 10);
  int start = 5;
  ht1632_putchar(start,1,'L'); 
  ht1632_putchar(start+6,1,'E'); 
  ht1632_putchar(start+12,1,'D'); 
  ht1632_putchar(start+18,1,'I'); 
  delay_ms(5000); 

  set_time(timebuf);
  disp_time(timebuf);

  // play tone to indicate we started
  play_tone(A5, 4); _delay_ms(5);
  play_tone(A5, 4); _delay_ms(10);

  while(1)
  {
    n = term_recvstr(BUF, BUFSIZE);

    if(n<0)
    {
      term_sendstr(PSTR("error occurred on uart\r\n"));
    }
    if(n>0)
    {
      MODE = term_check_mode(BUF,n);
      switch(MODE)
      {
        case MODE_TIMESET:
          mode_timeset(BUF,n);
          break;
        case MODE_DRAW:
          mode_draw();
          break;
        case MODE_INVALID:
        default:
          mode_msg(BUF,n);
          break;
      }
      // just to be paranoid
      stop_led_sync_timer();

      // hack to force time display
      minutes_last = minutes - 1;
    }

    if(last_button1state == 1)
    {
      demo_life(); 
      // if we come out of demo_life, we show time 
      set_time(timebuf);
      disp_time(timebuf);
    }

    if(seconds_last != seconds || last_buttonstate & BT_PRESS) 
    {
      seconds_last = seconds;

      if(minutes_last != minutes)
      {
        minutes_last = minutes;
        set_time(timebuf);
        disp_time(timebuf);
      }

      disp_bindots();
    }

    // fast cycle
    if(last_buttonstate & BT_HOLD)
    {
      while(!(PINC & (1<<BUTTON2)))
      {
        minutes++;
        update_datetime(1);
        set_time(timebuf);
        disp_time(timebuf);
        minutes_last = minutes;
        _delay_ms(20);
      }
      intcount = 0;
    }

    // while ((ASSR & (1<<OCR2BUB)) != 0) {} 
  }
}
