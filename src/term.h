#ifndef _TERM_H
#define _TERM_H

#define DEBUG            0

#define MODE_INVALID     0
#define MODE_DRAW        1
#define MODE_TIMESET     2
#define MODE_MSG         3

#define CMD_QUIT         0
#define CMD_BANK_EDIT    1
#define CMD_DRAW_LINE    2
#define CMD_DRAW_DOT     3
#define CMD_DRAW_RECT    4

void phexch(unsigned char c);
void phex(unsigned char c);
void phex16(unsigned int i);

void term_sendstr(const char *s);
int16_t term_recvstr(char *buf, uint8_t size);
uint8_t term_check_mode(const char *s, uint8_t n);

#endif
