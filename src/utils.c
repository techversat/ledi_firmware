#include <util/delay.h>

void delay_us(int us)
{
  int i;
  for (i=0; i<us; i++)
    _delay_us(1);
}

void delay_ms(int ms)
{
  int i;
  for (i=0; i<ms; i++)
    _delay_ms(1);
}


uint16_t exponent(uint16_t base, uint8_t n)
{
  uint16_t mult = 1;
  uint8_t i;
  for(i=0; i<n; i++)
    mult *= base;
  return mult;
}

void strtonum(char *p, uint8_t n, uint16_t *num)
{
  uint8_t i;
  uint16_t k;
  char c;

  if(*p=='\0') return;

  for(i=0; i<n; i++)
  {
    c = *p++;
    if(c=='0')
      k = 0;
    else
      k = (uint16_t) (c-'0') * exponent(10,n-1-i);
    *num += k;
  }
}


