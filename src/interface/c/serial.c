#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <time.h>
// #include "libfixmath/fix16.h"

int error;
typedef struct fstruct { 
  int fid;
  FILE *fstream;
} fobj;

int get_baudrate(int baudrate)
{
  int baudr;
  switch(baudrate)
  {
    case      50 : baudr = B50; break;
    case      75 : baudr = B75; break;
    case     110 : baudr = B110; break;
    case     134 : baudr = B134; break;
    case     150 : baudr = B150; break;
    case     200 : baudr = B200; break;
    case     300 : baudr = B300; break;
    case     600 : baudr = B600; break;
    case    1200 : baudr = B1200; break;
    case    1800 : baudr = B1800; break;
    case    2400 : baudr = B2400; break;
    case    4800 : baudr = B4800; break;
    case    9600 : baudr = B9600; break;
    case   19200 : baudr = B19200; break;
    case   38400 : baudr = B38400; break;
    case   57600 : baudr = B57600; break;
    case  115200 : baudr = B115200; break;
    case  230400 : baudr = B230400; break;
    case  460800 : baudr = B460800; break;
    case  500000 : baudr = B500000; break;
    case  576000 : baudr = B576000; break;
    case  921600 : baudr = B921600; break;
    case 1000000 : baudr = B1000000; break;
    default      : printf("invalid baudrate\n"); return(1); break;
  }
  return baudr;
}

int serial_setup(fobj *f, char *serialdev, int baudrate)
{
  static struct termios new_port_settings, old_port_settings;
  int baudr = get_baudrate(baudrate);

  f->fid = open(serialdev, O_RDWR | O_NOCTTY | O_NDELAY);
  if(f->fid==-1)
  {
    perror("unable to open comport ");
    return(1);
  }

  error = tcgetattr(f->fid, &old_port_settings);
  if(error==-1)
  {
    close(f->fid);
    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  new_port_settings.c_cflag = baudr | CS8 | CLOCAL | CREAD;
  new_port_settings.c_iflag = IGNPAR;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

  error = tcsetattr(f->fid, TCSANOW, &new_port_settings);
  if(error==-1)
  {
    close(f->fid);
    perror("unable to adjust portsettings ");
    return(1);
  }

  // associate a file stream with this file descriptor
  f->fstream = fdopen(f->fid, "r+"); 

  return(0);
}


int serial_read(int sid, unsigned char *buf, int size)
{
  int n;

#ifndef __STRICT_ANSI__                       /* __STRICT_ANSI__ is defined when the -ansi option is used for gcc */
  if(size>SSIZE_MAX)  size = (int)SSIZE_MAX;  /* SSIZE_MAX is defined in limits.h */
#else
  if(size>4096)  size = 4096;
#endif

  n = read(sid, buf, size);
  return(n);
}

int serial_write(int sid, unsigned char byte)
{
  int n;
  n = write(sid, &byte, 1);
  if(n<0)  return(1);
  return(0);
}

int serial_writen(int sid, unsigned char *buf, int size)
{
  int i,n = 0;
  // return(write(sid, buf, size));
  for(i=0; i<size; i++)
  {
    n = write(sid, &buf[i], 1);
    if(n<0) return(1);
    // for(n=0;n<1000000;n++);
  }
  return(0);
}



// serial buffer is not as fast as our loop. we will get lots of 0 byte returned
// need to keep reading one byte at a time until we hit newlines
int serial_readline2(int sid, unsigned char *buf, int size)
{
  int n = 0;
  int m = 0;
  unsigned char *ptr;
  ptr = buf;

  while(m<size)
  {
    n = read(sid, ptr, 1); // 1 byte at a time
    // end of file or nothing in buffer
    if(n==0) 
      continue;
    // error
    if(n<0)
    {
      printf("error reading!\n");  
      return n;
    }
    // if newline, no more read
    if(*ptr=='\r' || *ptr=='\n')
      break;

    m += n;
    // printf("n = %d, m = %d, size = %d, ch = %c\n", n, m, size, *ptr);
    ptr++; // advance the pointer
  }

  *(++ptr) = '\0';  // pad with null so that we are compatible

  // printf("returning with m = %d\n", m);
  return m;
}


int serial_readline(FILE* fstream, unsigned char *buf, int size)
{
  char *ptr;
  ptr = fgets(buf, size, fstream); 
  // if(ptr!=NULL && *(ptr-1) == '\r') *(ptr-1) = '\0';
  return ptr==NULL;
}


int main(int argc, char* argv[])
{
  int baud;
  // char buf[SSIZE_MAX];
  unsigned char buf[1024];
  fobj f;
  int n;

  if(argc < 4)
  {
    printf("usage: %s <serialdev> <baud> <msg>\n", argv[0]);
    return 1;
  }

  baud = atoi(argv[2]);  
  printf("opening serial port %s at %d\n", argv[1], baud);
      
  if(serial_setup(&f, argv[1], baud))
  {
    printf("ERROR: could not open serial port: %s\n", argv[1]);
    return 1;
  }

  serial_writen(f.fid, argv[3], strlen(argv[3]));
  serial_writen(f.fid, "\n", 1);
  return;

  while(1)
  {
    n = serial_read(f.fid, buf, 10); // let's try to read 10 bytes at a time 
    if(n>0) write(1, &buf[0], n);  

    // n = serial_readline(f.fstream, buf, 10); // let's try to read 10 bytes at a time 
    // if(!n) printf("line %s", buf);

    // n = serial_readline2(f.fid, buf, 100); // let's try to read 10 bytes at a time 
    // if(n>0) printf("%s\n", buf);
  }

  return 0;
}
