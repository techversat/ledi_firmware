#ifndef _HT1632_HEADER
#define _HT1632_HEADER

#include <avr/io.h>

// for other compilers without uint8_t, change this to char
#define byte  uint8_t
#define byte2 uint16_t

// modify here for more boards
#define N_PANELS    1
#define ALL_BOARDS  0b00000011   // bit mask for active display boards

#define OUTPUT 1
#define INPUT  0
// redefine these macros as necessary : can use ## to concat macro function params
#define PIN_MODE(X,Y,MODE)  if(MODE==1) X |= (1<<Y); else X &= ~(1<<Y)
#define PIN_SET_HIGH(X,Y)   X |= (1<<Y)
#define PIN_SET_LOW(X,Y)    X &= ~(1<<Y)
#define PIN_GET(X,Y)        ( X & (1<<Y) )

// configure stuff here
#define CS1_PORT    PORTD  // CS BoardSelect
#define CS2_PORT    PORTD  // CS BoardSelect
#define RD_PORT     PORTD  // Read Clock
#define WR_PORT     PORTD  // Write Clock
#define DAT_PORT    PORTD  // Write Data
#define DAT_PORT_RD PIND   // Read Data

#define CS1_REG     DDRD
#define CS2_REG     DDRD
#define RD_REG      DDRD
#define WR_REG      DDRD
#define DAT_REG     DDRD

#define CS1_PIN     PD2
#define CS2_PIN     PD5
#define RD_PIN      PD1
#define WR_PIN      PD3
#define DAT_PIN     PD4


/*
 * commands written to the chip consist of a 3 bit "ID", followed by
 * either 9 bits of "Command code" or 7 bits of address + 4 bits of data.
 */
#define HT1632_ID_CMD 4		      /* ID = 100 - Commands */
#define HT1632_ID_RD  6		      /* ID = 110 - Read RAM */
#define HT1632_ID_WR  5		      /* ID = 101 - Write RAM */
#define HT1632_ID_BITS (1<<2)   /* IDs are 3 bits */

#define HT1632_CMD_SYSDIS 0x00	/* CMD= 0000-0000-x Turn off oscil */
#define HT1632_CMD_SYSON  0x01	/* CMD= 0000-0001-x Enable system oscil */
#define HT1632_CMD_LEDOFF 0x02	/* CMD= 0000-0010-x LED duty cycle gen off */
#define HT1632_CMD_LEDON  0x03	/* CMD= 0000-0011-x LEDs ON */
#define HT1632_CMD_BLON   0x09	/* CMD= 0000-1001-x Blink On */
#define HT1632_CMD_BLOFF  0x08	/* CMD= 0000-1000-x Blink Off */
#define HT1632_CMD_SLVMD  0x10	/* CMD= 0001-00xx-x Slave Mode */
#define HT1632_CMD_MSTMD  0x14	/* CMD= 0001-01xx-x Master Mode */
#define HT1632_CMD_RCCLK  0x18	/* CMD= 0001-10xx-x Use on-chip clock */
#define HT1632_CMD_EXTCLK 0x1C	/* CMD= 0001-11xx-x Use external clock */
#define HT1632_CMD_COMS00 0x20	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_COMS01 0x24	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_COMS10 0x28	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_COMS11 0x2C	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_PWM    0xA0	/* CMD= 101x-PPPP-x PWM duty cycle */
#define HT1632_CMD_BITS (1<<7)

// used to pass data associated with plot func
struct plot_addr
{
  byte addr;
  byte csaddr;
  byte bitval;
};

void ht1632_selectboard(byte);
void ht1632_freeboard(void);
void ht1632_writebits(byte, byte);
byte ht1632_readbits(void);
void ht1632_sendcmd(byte, byte);
void ht1632_senddata(byte, byte, byte);
byte ht1632_readdata(byte, byte);
byte ht1632_readreplace(byte, byte, byte);
byte ht1632_setup(byte);
byte2 ht1632_xmax(void);
byte2 ht1632_ymax(void);
void ht1632_getaddr(byte,byte,byte,struct plot_addr*);
void ht1632_plot(byte, byte, byte, byte);
void ht1632_plotram(byte, byte, byte, byte);
void ht1632_editram(byte, byte, byte*, byte);
void ht1632_editram_char(byte, byte, char*, byte, byte);
void ht1632_snapram(void);
void ht1632_syncram(void);
void ht1632_syncram_fast(void);
void ht1632_clearram(void);
byte ht1632_getram(byte, byte);
void ht1632_clear(void);
void ht1632_wipe(byte, byte, byte);
void ht1632_putchar_col(byte, byte, char, byte colstart, byte colend);
void ht1632_putchar(byte, byte, char);
void ht1632_print(byte, byte, const char*);
void ht1632_sendbyte(byte x, byte y, byte dispbyte, byte clear);
void ht1632_scrollstr(const char* str, byte n, byte msec);
void ht1632_putchar2(byte x, byte y, char c, byte offset);
byte ht1632_chcol(unsigned char c, byte j);
char ht1632_whichch(unsigned char c);

#endif
