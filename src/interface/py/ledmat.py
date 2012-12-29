import re
import time
import sys

# use this as a mapping for fonts
_FONT = {
  '.' : [0, 0, 0x01, 0, 0], # dot
  ' ' : [0, 0, 0, 0, 0], # space!
  'a' : [0x3f, 0x48, 0x48, 0x48, 0x3f], # A
  'b' : [0x7f, 0x49, 0x49, 0x49, 0x36],
  'c' : [0x3e, 0x41, 0x41, 0x41, 0x22],
  'd' : [0x7f, 0x41, 0x41, 0x22, 0x1c],
  'e' : [0x7f, 0x49, 0x49, 0x49, 0x41],
  'f' : [0x7f, 0x48, 0x48, 0x48, 0x40],
  'g' : [0x3e, 0x41, 0x49, 0x49, 0x2e],  # G
  'h' : [0x7f, 0x08, 0x08, 0x08, 0x7f],
  'i' : [0x00, 0x41, 0x7f, 0x41, 0x00],
  'j' : [0x06, 0x01, 0x01, 0x01, 0x7e],
  'k' : [0x7f, 0x08, 0x14, 0x22, 0x41],
  'l' : [0x7f, 0x01, 0x01, 0x01, 0x01],
  'm' : [0x7f, 0x20, 0x10, 0x20, 0x7f],
  'n' : [0x7f, 0x10, 0x08, 0x04, 0x7f],
  'o' : [0x3e, 0x41, 0x41, 0x41, 0x3e],
  'p' : [0x7f, 0x48, 0x48, 0x48, 0x30],  # P
  'q' : [0x3e, 0x41, 0x45, 0x42, 0x3d],
  'r' : [0x7f, 0x48, 0x4c, 0x4a, 0x31],
  's' : [0x31, 0x49, 0x49, 0x49, 0x46],
  't' : [0x40, 0x40, 0x7f, 0x40, 0x40],
  'u' : [0x7e, 0x01, 0x01, 0x01, 0x7e],
  'v' : [0x7c, 0x02, 0x01, 0x02, 0x7c],
  'w' : [0x7f, 0x02, 0x04, 0x02, 0x7f],
  'x' : [0x63, 0x14, 0x08, 0x14, 0x63],
  'y' : [0x60, 0x10, 0x0f, 0x10, 0x60],
  'z' : [0x43, 0x45, 0x49, 0x51, 0x61],  # Z

  '0' : [0x3e, 0x45, 0x49, 0x51, 0x3e],  # 0 (pretty)
  '1' : [0x00, 0x10, 0x20, 0x7f, 0x00],
  '2' : [0x47, 0x49, 0x49, 0x49, 0x31],
  '3' : [0x42, 0x49, 0x59, 0x69, 0x46],
  '4' : [0x08, 0x18, 0x28, 0x7f, 0x08],
  '5' : [0x71, 0x49, 0x49, 0x49, 0x46],
  '6' : [0x3e, 0x49, 0x49, 0x49, 0x06],
  '7' : [0x40, 0x47, 0x48, 0x50, 0x60],
  '8' : [0x36, 0x49, 0x49, 0x49, 0x36],
  '9' : [0x30, 0x49, 0x49, 0x49, 0x3e], # 9

  'b0' : [0x7f, 0x41, 0x41, 0x41, 0x7f],  # 0 (block)
  'b1' : [0x00, 0x00, 0x00, 0x00, 0x7f],
  'b2' : [0x4f, 0x49, 0x49, 0x49, 0x79],
  'b3' : [0x49, 0x49, 0x49, 0x49, 0x7f],
  'b4' : [0x78, 0x08, 0x08, 0x08, 0x7f],
  'b5' : [0x79, 0x49, 0x49, 0x49, 0x4f],
  'b6' : [0x7f, 0x49, 0x49, 0x49, 0x4f],
  'b7' : [0x40, 0x40, 0x40, 0x40, 0x7f],
  'b8' : [0x7f, 0x49, 0x49, 0x49, 0x7f],  # 8
  'b9' : [0x79, 0x49, 0x49, 0x49, 0x7f]
 };

## {{{ http://code.activestate.com/recipes/113799/ (r1)
#
# bitfield manipulation
# not used. just ripped it for future use
#
class bf(object):
  """
  k = bf()
  k[3:7]=5
  print k[3]
  print k[5]
  k[7]=1
  print k[4:8]
  print int(k)
  """
  def __init__(self,value=0):
    self._d = value

  def __getitem__(self, index):
    return (self._d >> index) & 1 

  def __setitem__(self,index,value):
    value  = (value&1L)<<index
    mask   = (1L)<<index
    self._d  = (self._d & ~mask) | value

  def __getslice__(self, start, end):
    mask = 2L**(end - start) -1
    return (self._d >> start) & mask

  def __setslice__(self, start, end, value):
    mask = 2L**(end - start) -1
    value = (value & mask) << start
    mask = mask << start
    self._d = (self._d & ~mask) | value
    return (self._d >> start) & mask

  def __int__(self):
    return self._d



# Virtualizes the mapping of every led in our led matrix.
# This of this class as software canvas
class LedMat:
  def __init__(self, x=32, y=8):
    self.x = x
    self.y = y
    self.header = dict()
    self.cells  = [int('0',10) for i in range(int(x*y/4))];
  def setdelay(self, msec):
    """adds delay section in the command"""
    self.header['w'] = "%04d" % msec
  def setxoffset(self, x):
    self.header['x'] = "%02d" % x
  def setyoffset(self, y):
    self.header['y'] = "%02d" % y
  def setvalue(self, x, y, val):
    if x>=self.x or y>=self.y: return
    # if x>=self.x: x = self.x - 1
    # if y>=self.y: y = self.y - 1
  
    cellpos = int(x*4) + int(y/4)  # (x<<2) + (y>>2)
    bitval  = 8>>(y&3)
    currval = self.cells[cellpos]
    if(val):
      self.cells[cellpos] = currval | bitval
    else:
      self.cells[cellpos] = currval & ~bitval
  def getvalue(self, x, y):
    cellpos = int(x*4) + int(y/4)  # (x<<2) + (y>>2)
    bitval  = 8>>(y&3)
    currval = self.cells[cellpos]
    return currval & bitval
  def clear(self):
    self.cells  = [int('0',10) for i in range(int(self.x*self.y/4))];

  def cmd(self, stay=False):
    s = ""
    if len(self.header)>0:
      for k,v in self.header.iteritems():
        s += "%s%s" % (k,v)
      s += "-"
    for i in self.cells:
      s += "%1x" % (i)
    if stay:
      s += ":"
    return s

  def __getitem__(self,index):
    """assumes - column wise addressing, each bit""" 
    xpos = int(index / 4)
    ypos = int(index % 4)
    currval  = self.cells[xpos]
    if currval & (8>>(ypos&3)):
      return 1
    else: 
      return 0
  def __setitem__(self,index,val):
    if index>=self.x*self.y: index = self.x*self.y - 1
    xpos = int(index / 4)
    ypos = int(index % 4)
    currval  = self.cells[xpos]
    if(val):
      self.cells[xpos] = currval | (8>>(ypos&3))
    else:
      self.cells[xpos] = currval & ~(8>>(ypos&3))
  def __str__(self):
    s = ""
    for j in range(self.y):
      for i in range(self.x):      
        val = self.getvalue(i, j)
        if val==0:
          s += " ."
        else:
          s += " o"
      s += "\n"
    return s;


# draw on single canvas
class LedMatDraw:
  def __init__(self, lm=LedMat()):
    self.lm = lm

  def line(self, x1, y1, x2, y2):
    if ( (x1>=self.lm.x and x2>=self.lm.x) or (y1>=self.lm.y and y2>=self.lm.y) ):
      return

    dx = abs(x2 -x1)
    dy = abs(y2 -y1)
  
    p1x = 0
    p1y = 0 
    p2x = 0
    p2y = 0
  
    if (dx > dy):
      if (x2>x1):
        p1x=x1
        p1y=y1
        p2x=x2
        p2y=y2
      else: 
        p1x=x2
        p1y=y2
        p2x=x1
        p2y=y1
  
      y = p1y
      x = p1x
      count = 0
      increment = 1
      if p2y > p1y:
        increment =  1
      else:
        increment = -1

      for i in range(dx+1):
        count += dy;
        if (count > dx):
          count -= dx
          y += increment
        if (y>=0 and y<self.lm.y and x>=0 and x<self.lm.x):
          self.lm.setvalue(x,y,1);

        x += 1
        if (x>=self.lm.x):
          break
    else:
      if (y2>y1):
        p1x=x1
        p1y=y1
        p2x=x2
        p2y=y2
      else:
        p1x=x2
        p1y=y2
        p2x=x1
        p2y=y1

      y = p1y
      x = p1x
      count = 0
      increment = 1
      if p2x > p1x:
        increment =  1
      else:
        increment = -1
      for i in range(dy+1):
        count += dx
        if (count > dy):
          count -= dy
          x+= increment
        if (y>=0 and y<self.lm.y and x>=0 and x<self.lm.x):
          self.lm.setvalue(x,y,1)

        y+=1
        if (y>=self.lm.y):
          break



# wrapper interface to our LedMat canvas.
# this is the higher level drawing utility
class LedMatControl:
  def __init__(self, x=32, y=8):
    self.lms = list()
    self.x = x
    self.y = y
    self.LedMatPrev = None  # previous instance

  def textscrollcmds(self, text, y=0, delay=50):
    ls     = list()
    ncols  = len(text) * 6 # width of text in panel space

    for colpos in range(ncols+self.x): 
      xind  = self.x - min(colpos,self.x)
      start = int(max(0,colpos-self.x)/6) # char we want
      width = min(self.x/6, colpos/6)+1
      offset= max(0,colpos-self.x) % 6
  
      t = text[start:start+width]
      # print t
      l = self.textcmd(t, xind, y, offset, 0, delay)
      ls.append(l)
      print l.cmd()

    return ls


  def textcmd(self, text, x=0, y=0, xoffset=0, yoffset=0, delay=100):
    """convert text string to ledmat command"""
    l = LedMat(x=self.x, y=self.y)
    l.setdelay(delay)

    cols = list()
    xo = min(5, xoffset)
    yo = min(8, yoffset)
    end = self.x/6
    if xo>0: end+=1;
    for i in range( min(len(text), end) ):
      c    = text[i].lower()
      # cpos = ord(c.lower()) - ord('a') + 1
      if not _FONT.has_key(c):
        cols.extend( _FONT['.'] )
      else:
        cols.extend( _FONT[c] )
      if c != 0: cols.append(0x00)

    for j in range(xo,len(cols)):
      col = cols[j]
      for pos in range(yo,8):
        l.setvalue( x+j-xo, y+pos-yo, (col & 0x80>>pos) )
    return l


  def filecmd(self, filename, onchar='o', delay=500):
    """read from a file and map it to command"""
    ls = list()
    f  = open(filename, 'r') 
    lnum = 0
    for line in f:
      line = line.strip()
      m = re.match("^;",line)
      if m==None:
        lnum = 0
        next
      else:
        line = line[1:]
        if lnum==0:
          ls.append(LedMat(x=self.x, y=self.y))
          ls[-1].setdelay(delay)

        if lnum<8:
          [ls[-1].setvalue(j, lnum, int(line[j]==onchar)) for j in range(min(len(line),self.x))]
          lnum += 1 
        else:
          lnum = 0

    f.close()
    return ls


  def pbmfilecmd(self, filename, onchar='1', delay=1000):
    f = open(filename, 'r')
    w = self.x
    h = self.y
    chars = list()
    founddim = False
    for line in f:
      line = line.strip()
      if re.match("\d+\s+\d+",line):
        tok  = re.split("\s", line)
        w   = int(tok[0])
        h   = int(tok[1])
        founddim = True
        continue
      if founddim==False: continue
      if re.match("^#",line): continue

      chars.extend(line)

    f.close()
    tot = w*h     

    l = LedMat(x=self.x, y=self.y)
    l.setdelay(delay)
    n = 0
    for j in range(self.y):
      for i in range(self.x):
        if n>=tot: break
        l.setvalue(i,j,int(chars[n]=="1"))
        n += 1

    print l
    return l

  def rawcmd(self, raw):
    pass

