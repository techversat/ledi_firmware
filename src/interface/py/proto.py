#!/usr/bin/env python

import re
import time
import serial

def readuntil(s):
  line = s.readline()
  while line:
    if re.match("waiting",line)!=None: 
      return 
    print "out: %s" % line.strip()
    line = s.readline()


# ser = serial.Serial('/dev/ttyACM0', 38400, timeout=5)
ser = serial.Serial('/dev/ttyACM0', timeout=None)
ser.write("hi\n")
ser.write(":d\n")
# ser.flush()

readuntil(ser)

for i in range(5):
  # str = "x%02dy01w01000-%d;\n" % (i, i)
  readuntil(ser)
  # while ser.inWaiting() > 0: ser.flushInput()
  str = "%d;\n" % i+1
  print "sending: %s" % str.strip();
  ser.write(str)
  time.sleep(1)


readuntil(ser)
ser.close()

