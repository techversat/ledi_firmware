#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "utils.h"
#include "ht1632.h"
#include "font.h"

// Some of these variables will be configured during setup func
static const byte2 MAX_ROW     = 15;  // maximum LED rows 0::15
static const byte2 PANEL_WIDTH = 32;  // horizontal pixels per display #define MAX_PANELS  4
static byte2 NUM_CELLS;         // cells per 1 panel: 8 x 32 / 4 = 64,  since 4 LED's per cell
#define TOTAL_CELLS_MAX 128     // enough cells for 2 panels of 8x32

static byte2 MAX_COLUMN; 
static byte2 TOTAL_CELLS;       // total number of cells across all panels
static byte ht1632_shadowram[TOTAL_CELLS_MAX];  // copy of the display's RAM - all 2 panels
static byte HT1632_INITIALIZED = 0;

static byte2 USE_SHADOWRAM = 1;
#define NMAX(x,y) ((x>y) ? x : y )
#define NMIN(x,y) ((x<y) ? x : y )


/*
 * CS pins can be connected to different port. 
 * We need to check the bitmask and pull corresponding CS to low.
 * We look at just the lower 4 bits in the bitmask:
 *          CS 4 3 2 1
 *  0b 0 0 0 0 0 0 0 0
 */
void ht1632_selectboard(byte bitmask)
{
  if(bitmask & 0x01) PIN_SET_LOW(CS1_PORT, CS1_PIN);
  else               PIN_SET_HIGH(CS1_PORT, CS1_PIN);
  if(bitmask & 0x02) PIN_SET_LOW(CS2_PORT, CS2_PIN);
  else               PIN_SET_HIGH(CS2_PORT, CS2_PIN);
}

/*
 * Alias to ht1632_selectboard()
 */
void ht1632_freeboard()
{
  ht1632_selectboard(0);  // incurs cs pins to go high
}

/*
 * CLK goes low and data is set. CLK goes high and data is tx-ed.
 * MSB is transferred first. Goes from MSB -> LSB
 */
void ht1632_writebits(byte bits, byte firstbit)
{
  while (firstbit)
  {
    PIN_SET_LOW( WR_PORT, WR_PIN ); 
    if (bits & firstbit) {
      PIN_SET_HIGH( DAT_PORT, DAT_PIN );
    }
    else {
      PIN_SET_LOW( DAT_PORT, DAT_PIN );
    }
    PIN_SET_HIGH( WR_PORT, WR_PIN );
    firstbit >>= 1;
  }
}

/*
 * CLK goes low and data is set. CLK goes high and data is tx-ed.
 * MSB is transferred first. Goes from MSB -> LSB
 * Read 4 bit nybbles from chip. May be called repeatedly for
 * multiple sequential addresses.
 */
byte ht1632_readbits()
{
  byte bitIndex;
  byte retval = 0;
  PIN_MODE(DAT_REG, DAT_PIN, INPUT); // input mode
  for (bitIndex=4; bitIndex>=1; bitIndex--)
  {
    PIN_SET_LOW( RD_PORT, RD_PIN );
    PIN_SET_HIGH( RD_PORT, RD_PIN );
    if (PIN_GET( DAT_PORT_RD, DAT_PIN ) & 0xff)
      retval |= (1<<(bitIndex-1));
  }
  PIN_MODE(DAT_REG, DAT_PIN, OUTPUT); // back to output
  return retval;
}

/*
 * ht1632_sendcmd
 *  Send a command to the ht1632 chip.
 *  A command consists of a 3-bit "CMD" ID, an 8bit command, and
 *  one "don't care bit".
 *
 *   CS        CMD   Command bits            X  Free
 *   --------------------------------------------------
 *   CS_Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx CS_Free
 */ 
void ht1632_sendcmd(byte boardmask, byte command)
{
  ht1632_selectboard(boardmask);                    // Select chip
  ht1632_writebits(HT1632_ID_CMD, HT1632_ID_BITS); // send 3 bits of id: COMMMAND
  ht1632_writebits(command, HT1632_CMD_BITS);      // send the actual command
  ht1632_writebits(0, 1);                          // one extra dont-care bit in cmd 
  ht1632_freeboard();                              //done
}

/*
 * ht1632_senddata
 *  Send a nibble (4 bits) of data to a particular memory location of the
 *  ht1632. The command has 3 bit ID, 7 bits of address, and 4 bits of data.
 *
 *   CS        CMD   Address bits         Data        Free
 *   --------------------------------------------------------
 *   CS_Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 CS_Free
 *
 *  Note that the address is sent MSB first, while the data is sent LSB first!
 *  This means that somewhere a bit reversal will have to be done to get
 *  zero-based addressing of words and dots within words.
 *
 *  Currently, this routine only does single writes. You can do successive
 *  address writing with HT1632.
 */
void ht1632_senddata(byte boardmask, byte address, byte data)
{
  ht1632_selectboard(boardmask);                   // Select chip
  ht1632_writebits(HT1632_ID_WR, HT1632_ID_BITS);  // send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6);                 // Send address (7 bits)
  ht1632_writebits(data,    1<<3);                 // send data (4 bits)
  ht1632_freeboard();                              // done
}

// batch send
void ht1632_senddata_batch(byte boardmask, byte address, byte *data, byte size)
{
  uint8_t i;
  ht1632_selectboard(boardmask);                   // Select chip
  ht1632_writebits(HT1632_ID_WR, HT1632_ID_BITS);  // send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6);                 // Send address (7 bits)
  for(i=0; i<size; i++)
  {
    ht1632_writebits(data[i], 1<<3);               // send data (4 bits)
  }
  ht1632_freeboard();                              // done
}

byte ht1632_readdata(byte boardmask, byte address)
{
  byte retval;
  ht1632_selectboard(boardmask);                   // Select chip
  ht1632_writebits(HT1632_ID_RD, HT1632_ID_BITS);  // send ID: READ to RAM
  ht1632_writebits(address, 1<<6);                 // Send address
  retval = ht1632_readbits();                      // read 4 bits of data
  ht1632_freeboard();                              // deselect all display boards
  return retval;
}


byte ht1632_readreplace(byte boardmask, byte address, byte newvalue)
{
	byte retval;
  ht1632_selectboard(boardmask);                   // Select chip
  ht1632_writebits(HT1632_ID_RD, HT1632_ID_BITS);  // send ID: READ to RAM
  ht1632_writebits(address, 1<<6);			           // Send address
  retval = ht1632_readbits(); 					           // read 4 bits of data
  
	// this is more efficient than read-save-write by using less command 
	// and address sequences to replace data with newvalue
	ht1632_writebits(newvalue, 1<<3); 

  ht1632_freeboard();   								           // deselect all display boards
  return retval;
}

/*
 * ht1632_wipe
 *  Send value to all displays, and the shadow memory, and the snapshot
 *  memory.  This uses the "write multiple words" capability of
 *  the chipset by writing all 96 words of memory without raising
 *  the chipselect signal.
 */
void ht1632_wipe(byte boardmask, byte value, byte displayonly)
{
  byte i;
  // PANEL_WIDTH columns *2 bytes per column on each panel
  // wipes with value per column
  for (i=0; i<NUM_CELLS; i++)
    ht1632_senddata(boardmask, i, value);  // clear the display! 

  if(USE_SHADOWRAM && !displayonly)
    ht1632_clearram();
}

void ht1632_clear()
{
  ht1632_wipe(ALL_BOARDS, 0, 0);
}



/*
 * ht1632_setup
 */
byte ht1632_setup(byte useram)
{
  // byte tmp;

  if(HT1632_INITIALIZED)
    return HT1632_INITIALIZED;

  if(useram)
    USE_SHADOWRAM = 1;

	PIN_MODE(CS1_REG,CS1_PIN,OUTPUT);
	PIN_MODE(CS2_REG,CS2_PIN,OUTPUT);
	// PIN_MODE(RD_REG, RD_PIN, OUTPUT);
	PIN_MODE(WR_REG, WR_PIN, OUTPUT);
	PIN_MODE(DAT_REG,DAT_PIN,OUTPUT);

  // we first send it to all boards
  ht1632_selectboard(ALL_BOARDS);
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_SYSDIS);  // Disable system
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_COMS00);  // xy, PMOS drivers, 32OUT 8COMS, 8x32
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_MSTMD);   // Master Mode
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_RCCLK);   // Clock on
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_SYSON);   // System on
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_LEDON);   // LEDs on
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_PWM);     // PWM on
  ht1632_sendcmd(ALL_BOARDS, HT1632_CMD_PWM | 0x0F);  // PWM max brightness (16 levels, from 0x0 - 0xF)

  /*
  for (N_PANELS=0; N_PANELS<4; N_PANELS++)
  {
    tmp = ht1632_readdata(1<<N_PANELS, 1);

    if(tmp == 0x0f)
    {
      break;
    }
  }
  */

  NUM_CELLS   = 2*PANEL_WIDTH;   // 8 x 32 / 4 = 64,  number of cells per panel
  MAX_COLUMN  = (N_PANELS * PANEL_WIDTH) -1;
  TOTAL_CELLS = N_PANELS * NUM_CELLS; // 96 banks (of 4 bits) for 24 x 16 matrix
  
  ht1632_clear();

  HT1632_INITIALIZED = 1;
  
  return HT1632_INITIALIZED;
}

byte2 ht1632_xmax()
{
  return MAX_COLUMN+1;
}

byte2 ht1632_ymax()
{
  return MAX_ROW+1;
}


// for 8 x 32 display
void ht1632_getaddr(byte x, byte y, byte wrap, struct plot_addr* pa)
{
  if(x > MAX_COLUMN)
    x = (wrap) ? x % MAX_COLUMN : MAX_COLUMN;  // wrap : bounds
  if(y > MAX_ROW)
    y = (wrap) ? y % MAX_ROW : MAX_ROW;        // wrap : bounds

  // mult x position by 2, divide y pos by 4
  pa->addr   = (x<<1) + (y>>2);               // compute which memory word this is in
  pa->csaddr = ((x%PANEL_WIDTH)<<1) + (y>>2); // compute which panel this is in
  pa->bitval = 8>>(y&3);                      // compute which bit will need to be set
}


/* 
 * Only update our ram. This bails if we are not using shadowram.
 */
void ht1632_plotram(byte x, byte y, byte val, byte wrap)
{
  byte displayByte;
  struct plot_addr pa;

  if(!USE_SHADOWRAM)
    return;

  ht1632_getaddr(x, y, wrap, &pa);
  displayByte = ht1632_shadowram[pa.addr];

  if (val)
    displayByte |= pa.bitval;
  else
    displayByte &= ~pa.bitval;

  ht1632_shadowram[pa.addr] = displayByte;
}

// uses the raw byte values. Each byte is broken into 2 words
void ht1632_editram(byte x, byte y, byte *data, byte size)
{
  byte word;
  byte *pos;
  byte ii,shift;
  byte2 i,j;
  struct plot_addr pa;

  ht1632_getaddr(x, y, 0, &pa);
  pos = &ht1632_shadowram[pa.addr];
  j = TOTAL_CELLS - pa.addr;

  // size is in bytes, but we need to iterate per word (4bits)
  for(i=0; i<size*2; i++)
  {
    // if we overshoot to the end of the buffer
    if(j - i < 0)
      break;

    ii    = i / 2;
    shift = (i%2) * 4; 
    word  =  0x0F & (data[ii] >> shift);
    *pos++ = word;
  }
}

uint8_t chartohex(char c)
{
  c = (c >= 'a' && c <= 'f') ? 0x0a + c - 'a' :
      (c >= 'A' && c <= 'F') ? 0x0a + c - 'A' :
      (c >= '0' && c <= '9') ? c - '0' :
      0;
  return c;
}

// converts each char value 0-f and uses it as is.
void ht1632_editram_char(byte x, byte y, char *data, byte size, byte update)
{
  byte *pos;
  byte2 i,j;
  struct plot_addr pa;

  ht1632_getaddr(x, y, 0, &pa);
  pos = &ht1632_shadowram[pa.addr];
  j = TOTAL_CELLS - pa.addr;
  if(size > TOTAL_CELLS)
    size = TOTAL_CELLS;

  // size is in bytes, but we need to iterate per word (4bits)
  for(i=0; i<size; i++)
  {
    // if we overshoot to the end of the buffer
    if(j <= i)
      break;

    if(update==1)
      *pos++ |= chartohex(data[i]);
    else
      *pos++  = chartohex(data[i]);
  }
}



/*
 * plot a point on the display, with the upper left hand corner
 * being (0,0), and the lower right hand corner being (23, 15).
 * Note that Y increases going "downward" in contrast with most
 * mathematical coordiate systems, but in common with many displays
 * No error checking; bad things may happen if arguments are out of
 * bounds!  (The ASSERTS compile to nothing by default
 */
void ht1632_plot(byte x, byte y, byte val, byte wrap)
{
  byte displayByte;
  struct plot_addr pa;
  ht1632_getaddr(x, y, wrap, &pa);

  if(USE_SHADOWRAM)
    displayByte = ht1632_shadowram[pa.addr];
  else
    displayByte = ht1632_readdata((1 << (x/PANEL_WIDTH) ), pa.csaddr); // read existing LED data

  // modify the display byte
  if (val)
    displayByte |= pa.bitval;
  else
    displayByte &= ~pa.bitval;

  if(USE_SHADOWRAM)
    ht1632_shadowram[pa.addr] = displayByte;
  // 0b10100000 == 32 + 128 == 160  (overshoots)
  // 0b10100000 >> 5 == 0b101 == 0b101 << 1 == 0b1010, 2nd and 4th disp??
  // 0b00010111 (23) >> 5 == 0b000  => 1<<0b000 == 0b001, 1st disp
  // 0b00011000 (24) >> 5 == 0b000  => 1<<0b000 == 0b001, 1st disp (but should be 2nd)
  // 5 bits == 31 - this shifting is designed for 8 x 32 matrix

  // we don't send to the display if we write ram only
  // ht1632_senddata((1 << (x >> 5)), addr, displayByte);
  ht1632_senddata(1<<(x/PANEL_WIDTH), pa.csaddr, displayByte);
}


/*
 * ht1632_snapram    
 *  Copy the shadow ram into the snapshot ram (the upper bits)
 *  This gives us a separate copy so we can plot new data while
 *  still having a copy of the old data.  snapshotram is NOT
 *  updated by the plot functions (except "clear")
 */
void ht1632_snapram()
{
  byte i;
  for (i=0; i<TOTAL_CELLS; i++) {
    ht1632_shadowram[i] = (ht1632_shadowram[i] & 0x0F) | ht1632_shadowram[i] << 4;  // Use the upper bits
  }
}

/*
 * Syncs the content of the ram to led. Clears the display first.
 * NUM_CELLS indicates total mem banks in each panel.
 */
void ht1632_syncram()
{
  byte i;
  ht1632_wipe(ALL_BOARDS, 0, 1);
  ht1632_snapram(); // save the state.

  for (i=0; i<TOTAL_CELLS; i++)
  {
    ht1632_senddata(1<<(i/NUM_CELLS), i%NUM_CELLS, ht1632_shadowram[i]);
  }
}

void ht1632_syncram_fast()
{
  // batch update in one go per display
  //  * does not clears the board prior to sync-ing
  //  * does not save the snap of the ram 
  ht1632_senddata_batch(0x01, 0, &ht1632_shadowram[0],  64);
  if(N_PANELS>1)
    ht1632_senddata_batch(0x02, 0, &ht1632_shadowram[64], 64);
}

/*
 * Clear the lower half of each bank
 */
void ht1632_clearram()
{
  byte i;
  for (i=0; i<TOTAL_CELLS; i++)
    ht1632_shadowram[i] = ht1632_shadowram[i] & 0xF0;
}


/*
 * get_snapshotram
 *  get a pixel value from the snapshot ram (instead of
 *  the actual displayed (shadow) memory
 */
byte ht1632_getram(byte x, byte y)
{
  byte addr, bitval;

  bitval = 128>>(y&3);       // user upper bits!
  addr   = (x<<1) + (y>>2);  // compute which memory word this is in
  if (ht1632_shadowram[addr] & bitval)
    return 1;
  return 0;
}

/*
void ht1632_putchar(byte x, byte y, char c)
{
  ht1632_putchar_col(x, y, c, 0, 5);
}

void ht1632_putchar_col(byte x, byte y, char c, byte colstart, byte colend)
{
  byte col;
  byte row;
  byte dots;
  if ( (c >= 'A' && c <= 'Z') ||
    ( c >= 'a' && c <= 'z') ) {
    c &= 0x1F;   // A-Z maps to 1-26
  }
  else if (c >= '0' && c <= '9') {
    c = (c - '0') + 27;
  }
  else if (c == ' ') {
    c = 0; // space
  }

  if(colstart>5) colstart=4;
  if(colend>5)   colend=5;
  for (col=colstart; col<colend; col++) {
    dots = pgm_read_byte_near(&myfont[(byte)c][(byte)col]);
    for (row=0; row < 7; row++) {
      if (dots & (64>>row))          // only 7 rows.
        ht1632_plot(x+col, y+row, 1, 0);
      else
        ht1632_plot(x+col, y+row, 0, 0);
    }
  }
}
*/

void ht1632_putchar(byte x, byte y, char c)
{
  byte col;
  byte row;
  byte dots;
  // c = ht1632_whichch(c);

  for (col=0; col< 5; col++)
  {
    dots = ht1632_chcol(c, col);
    for (row=0; row < 7; row++)
    {
      if (dots & (0x01<<row))          // only 7 rows.
        ht1632_plot(x+col, y+row, 1, 0);
      else
        ht1632_plot(x+col, y+row, 0, 0);
    }
  }
}


void ht1632_print(byte x, byte y, const char* str)
{
  char c;
  byte offset = 0;
  while(c = *str++)
  {
    ht1632_putchar(x+offset, y, c);
    offset += 6;
  }
}

void ht1632_sendbyte(byte x, byte y, byte dispbyte, byte clear)
{
  struct plot_addr pa;
  ht1632_getaddr(x, y, 0, &pa);
  if(clear)
    ht1632_senddata(1<<(x/PANEL_WIDTH), pa.csaddr, 0x00);

  ht1632_senddata(1<<(x/PANEL_WIDTH), pa.csaddr, dispbyte);
}


char ht1632_whichch(unsigned char c)
{
  if ( (c >= 'A' && c <= 'Z') ||
    ( c >= 'a' && c <= 'z') ) {
    c &= 0x1F;   // A-Z maps to 1-26
  }
  else if (c >= '0' && c <= '9') {
    c = (c - '0') + 27; // 37;  // 27;
  }
  else if (c == ' ') {
    c = 0; // space
  }
  return c;
}

byte ht1632_chcol(unsigned char c, byte j)
{
  if(j > 4)
    j = 0;
  return pgm_read_byte_near( &myfont[c - 32][j] );
}


void ht1632_putchar2(byte x, byte y, char c, byte offset)
{
  byte col;
  byte row;
  byte dots;
  byte tf;
  // c = ht1632_whichch(c);

  for (col=offset; col< 5; col++)
  {
    dots = ht1632_chcol(c, col);
    for (row=0; row < 7; row++)
    {
      // tf = (dots & (64>>row)) ? 1 : 0;          // only 7 rows.
      tf = (dots & (0x01<<row)) ? 1 : 0;          // only 7 rows.
      ht1632_plotram(x+col-offset, y+row, tf, 0);
    }
  }
}


#define DELAY_CONSTANT 30.0
void ht1632_scrollstr(const char* str, byte n, byte msec)
{
  int16_t x, xind, i, o, ii, j,pos;
  int8_t  width = MAX_COLUMN+1;
  int16_t txtw = n * 6;
  int16_t span = txtw + width;
  char* ptr;
  char ch;

  for(x=0; x<span; x++)
  {
    xind = width - NMIN(x, width);     // panel scroll position
    i    = NMAX(0, x - width) / 6;     // str start position
    o    = NMIN(width/6, x/6) + 1;     // length of substr
    o    = NMIN(o, n-i);
    ii   = NMAX(0, x-width) % 6;       // front char offset
    ptr  = (char*) &str[i];

    ht1632_clear();

    j = 0;
    pos = xind;
    while( (ch = *ptr++) && (j < o) )
    {
      if(ch=='\n') break;
      if(pos>MAX_COLUMN) break;
      if(j>=n) break;

      ht1632_putchar2(pos, 1, ch, (j==0) ? ii : 0);
      pos += (j==0) ? 6 - ii : 6;
      j++;
    }

    ht1632_syncram();
    delay_ms(msec);
  }

  ht1632_clear();
}

