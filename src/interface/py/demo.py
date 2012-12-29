#!/usr/bin/env python

import re
import time
import sys
import optparse
import random
import serial
from ledmat import *
from cube import *


def printn(st):
  sys.stdout.write(st)

def readline(s):
  line = ""
  while 1:
    n = s.inWaiting()
    if n == 0:
      return line
    c = s.read(1)
    if c == '\r':
      s.read(1) # discard the following '\n'
      return line
    line += c


def readecho(s):
  # time.sleep(0.01)
  n = 1
  while n > 0:
    line = readline(s)
    n    = s.inWaiting()
    if len(line)==0:
      return
    printn("out: [%04d] %s\n" % (n,line))


### artificial delay necessary to make it work.... sigh...
# this is due to the inaccuracy of the internal oscillator,
# which needs calibration to make it within +/-1%. By default
# it is +/-10%. Pretty sad, when you start running the clock at 8MHz
def writeserial(s,ch):
  n = 0
  for i in range(len(ch)):
    n = n + s.write(ch[i])
    # time.sleep(0.0008) # delay necessary???
  return n


def senddata(s, msg, doflush):
  # n = writeserial(s,msg+"\n")
  n = s.write(msg+"\n")
  if n==0:
    print "error: no data written"
  if doflush:
    n = s.inWaiting()
    while n>0:
      n = s.inWaiting()
      s.flushInput()
      s.flush()


def demo_hscan(ser):
  l = range(32)
  for i in l:
    msg = "x%02dy00w0100-ff;" % i
    senddata(ser,msg, True)
    # time.sleep(0.100)
  l.reverse()
  for i in l:
    msg = "x%02dy00w0100-ff;" % i
    senddata(ser,msg, True)
    # time.sleep(0.100)

def demo_vdraw(ser):
  # very inefficient3
  l = range(8)
  for i in l:
    msg = ""
    for j in range(32):
      bitval = 2**(3-(i%4))
      msg = "x%02dy%02dw0000-%d:" % (j, i, bitval)
      senddata(ser,msg, True)

def demo_vscan2(ser):
  l  = range(8)
  l2 = range(8); l2.reverse()
  l.extend(l2)
  print l
  for i in l:
    l = LedMat(x=32, y=8)
    l.setdelay(50)
    [ l.setvalue(j,i,1) for j in range(32)]
    senddata(ser, l.cmd(), True)

def demo_random(ser):
  x = y = dx = dy = 0
  xmax = 32
  ymax = 8
  # create and destroys LedMat 1000 times.
  # let's see how slow this is
  for i in range(1000):
    l = LedMat()
    change = random.randint(0,32)
    l.setdelay(10)
    if   change == 10: dx = -dx
    elif change == 11: dy = -dy
    elif change == 12: dx = dy = 0
    elif change == 13: dx = dy = 1
    elif change == 14: dx = dy = -1
    x = x + dx;
    y = y + dy;
    if x > xmax:
      dx = -1
      x = xmax
    elif x < 0:
      x = 0
      dx = 1
    if y > ymax:
      y = ymax
      dy = -1
    elif y < 0:
      y = 0
      dy = 1
    # hah! we just set single value but transmit ~200 bytes!
    l.setvalue(x,y,1)
    senddata(ser, l.cmd(), True)


def demo_draw(ser):
  print "blah"
  lc = LedMatControl()
  ll = lc.filecmd("data.txt")
  for l in ll:
    senddata(ser, l.cmd(), True)


def demo_scroll(ser):
  lc = LedMatControl()
  ls = lc.textscrollcmds("scrolling txt test by jbremnant it can display lots of info", 3, 50)
  i = 0
  for l in ls:
    l.setvalue(i%32,15,1)
    l.setvalue(32-i%32,0,1)
    senddata(ser, l.cmd(), True)
    i += 1

def demo_img(ser):
  lc = LedMatControl()
  p = lc.pbmfilecmd("imgs/title.pbm")
  senddata(ser, p.cmd(True), True)


def demo_fps(ser):
  lc = LedMatControl()
  # calculate 1000 iterations
  fpsstr = "calc"
  start = time.time();  
  for i in range(3000):
    l = None
    clear = False
    if i>0 and i%100==0:
      fps = 100 / (time.time() - start)
      start = time.time()
      fpsstr = "%4.2f" % fps
      clear = True
    else:
      clear = False 
      
    l = lc.textcmd(fpsstr, 8, 0, 0, 0);
    l.setdelay(0)
    l.setvalue(random.randint(0,47), random.randint(11,15), 1)
    senddata(ser, l.cmd(not clear), True)


def demo_cube(ser):
  TWO_PI    = 6.28
  cubeWidth = 20 # Cube size
  Angx      = 0
  AngxSpeed = 0.017 # rotation (angle+speed) around X-axis
  Angy      = 0
  AngySpeed = 0.022 # rotation (angle+speed) around Y-axis

  cube = Cube()
  cube.make(cubeWidth);

  for i in range(2000):
    Angx+=AngxSpeed
    Angy+=AngySpeed
    if (Angx>=TWO_PI):
      Angx-=TWO_PI
    if (Angy>=TWO_PI):
      Angy-=TWO_PI

    cube.rotate(Angx, Angy)
    # Draw cube
    lm = LedMat()
    cube.draw(lm)

    lm.setdelay(10)
    senddata(ser, lm.cmd(), True)
    print lm
    lm.clear()



funcs = dir()

def main():
  parser = optparse.OptionParser(
    usage = "%prog [options]",
    description = "termint - my stupid interface to ledmetrix over serial port."
  )
  parser.add_option("-d", "--device",
    dest = "device",
    help = "either /dev/ttyACM0 or..",
    default = "/dev/ttyUSB0"
  )
  (options, args) = parser.parse_args()
  
  ser = serial.Serial(options.device, 38400, timeout=None)
  # clean slate
  ser.flushInput()
  ser.flushOutput()
  # senddata(ser,":q\r\n",False)
  # senddata(ser,":d\r\n",False)

  doexit = 0;

  while not doexit:
    printn("lm> ")
    strin = raw_input()
  
    if len(strin) == 0:
      continue

    tok  = re.split("\s", strin)
    if len(tok) > 1:
      cmd  = tok[0]

      if cmd == "demo": 
        arg  = tok[1]
        func = cmd + "_" + arg
        try:
          eval(func)(ser)  # all demo functions take serial object as arg
        except:
          print "could not eval func: " + func
          continue
      else:
        print "unsupported cmd"
        continue 
    elif len(tok)==1 and tok[0]=="demo":
      print "choose demo function from: "
      print [re.sub("_"," ",i) for i in funcs if re.match("demo",i)] 
    else:
      senddata(ser,strin, False)
  
    readecho(ser)
  
  ser.close()


if __name__ == '__main__':
    main()
