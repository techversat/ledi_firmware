#!/usr/bin/env python

import re
import sys
import time
import serial

def printn(st):
  sys.stdout.write(st)

def readuntil(s):
  # line = s.readline()
  n = s.read()
  while len(n)>0:
    printn(n)
    n = s.read()

def writeserial(s,ch):
  for i in range(len(ch)):
    s.write(ch[i])
    # time.sleep(0.0007) # delay necessary???

# ser = serial.Serial(port='/dev/ttyUSB0', baudrate=38400, timeout=1, xonxoff=True)
# ser = serial.Serial(port='/dev/ttyUSB0', baudrate=38400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=None)
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=38400,timeout=None)
ser.flush()
#ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=None)
# ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=None)
writeserial(ser, "this is pain in the ass\n")
ser.flush()

ser.flush()
time.sleep(0.1)
# writeserial(ser,":q\r\n");
writeserial(ser, ":d\n")
time.sleep(0.1)
writeserial(ser, "x01y01w1000-ff1248f;x01y02w0500-1111:\n")

readuntil(ser)

"""
for i in range(5):
  # str = "x%02dy01w01000-%d;\n" % (i, i)
  readuntil(ser)
  # while ser.inWaiting() > 0: ser.flushInput()
  str = "%d;\n" % i+1
  print "sending: %s" % str.strip();
  ser.write(str)
  time.sleep(1)


readuntil(ser)
"""

ser.close()

