TARGET=test_timer1
PORT=/dev/ttyUSB0

MCUID = m328p
MCU = atmega328p
CPU_FREQ = 8000000UL
TWI_FREQ = 400000 
UPLOAD_RATE = 115200  # 57600
AVRDUDE_PROGRAMMER = stk500

GCCFLAGS=-Os -Wall -mmcu=$(MCU) -DCPU_FREQ=$(CPU_FREQ) -DF_CPU=$(CPU_FREQ) -DTWI_FREQ=$(TWI_FREQ) -I../../
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm

AVRDUDE = avrdude
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_FLAGS=-V -F -c avr109 -p $(MCUID) -P $(PORT) -b $(UPLOAD_RATE) $(AVRDUDE_WRITE_FLASH)
AVRDUDE_FLAGS_ISP=-V -F -P usb -c avrispmkII -p $(MCUID) $(AVRDUDE_WRITE_FLASH)

all:	$(TARGET).hex
all2:	$(TARGET)_sync.hex
upload: $(TARGET)-uploadisp
upload2: $(TARGET)-uploadisp2

clean:
	rm *.o *.hex

calibration.o: ../../calibration.c
	cd ../.. && avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o calibration.o calibration.c

uart.o: ../../uart.c 
	cd ../.. && avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o uart.o uart.c

utils.o: ../../utils.c 
	cd ../.. && avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o utils.o utils.c

$(TARGET).hex: $(TARGET).c uart.o calibration.o
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o $(TARGET).o ../../calibration.o ../../uart.o ../../utils.o $(TARGET).c
	avr-objcopy -j .text -j .data -O ihex $(TARGET).o $(TARGET).hex
	avr-size $(TARGET).o
	
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

