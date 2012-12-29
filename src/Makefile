# In order to use internal RC to run at 8MHz (max internal clock speed)
#   avrdude -F -p m168 -P usb -c avrispmkII -U lfuse:w:0xE2:m
#   avrdude -F -p m328p -P usb -c avrispmkII -U lfuse:w:0xE2:m
#
# To read the fuse settings:
#   ATMEGA168:  avrdude -F -p m168 -P usb -c avrispmkII -U lfuse:r:-:h -U hfuse:r:-:h
#   ATMEGA328P: avrdude -F -p m328p -P usb -c avrispmkII -U lfuse:r:-:h -U hfuse:r:-:h
#
# Good source for this info:
# 
# lfuse: 0xe2 
# 			 0xe6  for 16MHz external crystal
# hfuse: 0xdf  0b1001 1011
#
# To change clock setting on the programmer:
# jbkim@homelnx:/nas/dev/c/embedded/nerdkit/projects/clockdisplay$ avrdude -V -F -P usb -c avrispmkII -p m8 -u -t -s
#
# avrdude: AVR device initialized and ready to accept instructions
#
# Reading | ################################################## | 100% 0.05s
#
# avrdude: Device signature = 0x1e940b
# avrdude: Expected signature for ATMEGA8 is 1E 93 07
# avrdude> parms
# >>> parms
# Vtarget         : 0.0 V
# SCK period      : 32.12 us
# avrdude> sck 10
# >>> sck 10
# avrdude> parms
# >>> parms
# Vtarget         : 0.0 V
# SCK period      : 10.37 us
# avrdude> quit
# >>> quit
#
# 	http://www.sparkfun.com/commerce/tutorial_info.php?tutorials_id=95

TARGET=main
PORT=/dev/ttyUSB0

# MCUID = m168
MCUID = m328p
MCU = atmega328p
# CPU_FREQ = 16000000
# CPU_FREQ = 14745600
CPU_FREQ = 8000000
TWI_FREQ = 400000 
UPLOAD_RATE = 115200  # 57600
AVRDUDE_PROGRAMMER = stk500

# GCCFLAGS=-g -Os -Wall -mmcu=$(MCU) -DCPU_FREQ=$(CPU_FREQ) -DF_CPU=$(CPU_FREQ) -DTWI_FREQ=$(TWI_FREQ)
GCCFLAGS=-Os -Wall -mmcu=$(MCU) -DCPU_FREQ=$(CPU_FREQ) -DF_CPU=$(CPU_FREQ) -DTWI_FREQ=$(TWI_FREQ)
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm

AVRDUDE = avrdude
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_FLAGS=-V -F -c avr109 -p $(MCUID) -P $(PORT) -b $(UPLOAD_RATE) $(AVRDUDE_WRITE_FLASH)
AVRDUDE_FLAGS_ISP=-V -F -P usb -c avrispmkII -p $(MCUID) $(AVRDUDE_WRITE_FLASH)
# -c $(AVRDUDE_PROGRAMMER) -b $(UPLOAD_RATE)

all:	$(TARGET).hex
all2:	$(TARGET)_sync.hex
upload: $(TARGET)-uploadisp
upload2: $(TARGET)-uploadisp2

clean:
	find *.o *.hex | xargs rm

calibration.o: calibration.c
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o calibration.o calibration.c

utils.o: utils.c 
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o utils.o utils.c

ht1632.o: ht1632.c 
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o ht1632.o ht1632.c

uart.o: uart.c 
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o uart.o uart.c

uartsync.o: uartsync.c 
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -DUSE_UARTSYNC=1 -c -o uartsync.o uartsync.c

term.o: term.c 
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o term.o term.c

termsync.o: term.c 
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -DUSE_UARTSYNC=1 -c -o termsync.o term.c

$(TARGET).hex: $(TARGET).c utils.o ht1632.o uart.o term.o calibration.o
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o $(TARGET).o calibration.o utils.o ht1632.o uart.o term.o $(TARGET).c
	avr-objcopy -j .text -j .data -O ihex $(TARGET).o $(TARGET).hex
	avr-size $(TARGET).o

$(TARGET)_sync.hex: $(TARGET).c utils.o ht1632.o uartsync.o termsync.o calibration.o
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -DUSE_UARTSYNC=1 -o $(TARGET)_sync.o calibration.o utils.o ht1632.o uartsync.o termsync.o $(TARGET).c
	avr-objcopy -j .text -j .data -O ihex $(TARGET)_sync.o $(TARGET)_sync.hex
	avr-size $(TARGET)_sync.o

	
$(TARGET).ass:	$(TARGET).hex
	avr-objdump -S -d $(TARGET).o > $(TARGET).ass
	
$(TARGET)-uploadbl:	$(TARGET).hex
	./pulsedtr.py $(PORT)
	$(AVRDUDE) $(AVRDUDE_FLAGS)

$(TARGET)-uploadisp:	$(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS_ISP)

tinyisp:	$(TARGET).hex
	$(AVRDUDE) -V -F -c usbtiny -p $(MCUID) -U flash:w:$(TARGET).hex

$(TARGET)-uploadisp2:	$(TARGET)_sync.hex
	$(AVRDUDE) -V -F -P usb -c avrispmkII -p $(MCUID) -U flash:w:$(TARGET)_sync.hex



