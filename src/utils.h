#ifndef _UTIL_HEADER
#define _UTIL_HEADER

void delay_us(int us);
void delay_ms(int ms);

// some convenience routines
uint16_t exponent(uint16_t base, uint8_t n);
void strtonum(char *p, uint8_t i, uint16_t *num);

#endif
