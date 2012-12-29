# Ripped and python-ized from + PeggyCube - LED rotating wired cube - 05 june 2008
# Credits :
#   - code by Micha Zancan & Julien 'v3ga' Gachadoat
#   - conversion by jbremnant

import math
import ledmat

# ------------------------------------------------------------
# Objects
# ------------------------------------------------------------
# Peggy2 frame1,frame2; # Drawing surfaces
# The cube is defined before the setup method


# ------------------------------------------------------------
# struct Vertex
# ------------------------------------------------------------
class Vertex:
  def __init__(self,x=0,y=0,z=0):
    self.x = 0
    self.y = 0
    self.z = 0
    self.set(x,y,z);

  def set(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

# ------------------------------------------------------------
# struct EdgePoint
# ------------------------------------------------------------
class EdgePoint:
  def __init__(self, x=0,y=0):
    self.x = 0
    self.y = 0
    self.set(x,y)
    self.visible = False

  def set(self, a, b):
    self.x = a
    self.y = b

# ------------------------------------------------------------
# struct Point
# ------------------------------------------------------------
class Point:
  def __init__(self, x=0,y=0):
    self.x = 0
    self.y = 0
    self.set(x,y)

  def set(self, a, b):
    self.x = a
    self.y = b

# ------------------------------------------------------------
# struct squareFace
# ------------------------------------------------------------
class SquareFace:
  def __init__(self, a=-1, b=-1, c=-1, d=-1):
    self.sommets = [0 for i in range(4)]
    self.ed      = [0 for i in range(4)]
    self.length  = 4
    self.set(a,b,c,d)

  def __len__(self):
    return self.length

  def set(self, a, b, c, d):
    self.sommets[0]=a;
    self.sommets[1]=b;
    self.sommets[2]=c;
    self.sommets[3]=d;

# ------------------------------------------------------------
# struct Cube
# ------------------------------------------------------------
class Cube:
  def __init__(self):
    self.local   = [Vertex() for i in range(8)] # Local vertices
    self.aligned = [Vertex() for i in range(8)] # Camera aligned vertices
    self.screen  = [Point() for i in range(8)]  # On-screen projected vertices
    self.face    = [SquareFace() for i in range(6)] # Faces
    self.edge    = [EdgePoint() for i in range(12)] # Edges
    # ModelView matrix
    self.nbEdges = 0
    self.m00 = 0
    self.m01 = 0
    self.m02 = 0
    self.m10 = 0
    self.m11 = 0
    self.m12 = 0
    self.m20 = 0
    self.m21 = 0
    self.m22 = 0

  # constructs the cube
  def make(self,w):
    self.nbEdges = 0;

    self.local[0].set(-w,w,w);
    self.local[1].set(w,w,w);
    self.local[2].set(w,-w,w);
    self.local[3].set(-w,-w,w);
    self.local[4].set(-w,w,-w);
    self.local[5].set(w,w,-w);
    self.local[6].set(w,-w,-w);
    self.local[7].set(-w,-w,-w);  

    self.face[0].set(1,0,3,2);
    self.face[1].set(0,4,7,3);
    self.face[2].set(4,0,1,5);
    self.face[3].set(4,5,6,7);
    self.face[4].set(1,2,6,5);
    self.face[5].set(2,3,7,6);

    for f in range(6):
      for i in range( len(self.face[f]) ):
        j = i
        if i:
          j = i - 1
        else:
          j = len(self.face[f])-1

        self.face[f].ed[i]= self.findEdge(
          self.face[f].sommets[i],
          self.face[f].sommets[j]) 

  # finds edges from faces
  def findEdge(self, a, b):
    i = 0
    for i in range(self.nbEdges):
      if( (self.edge[i].x==a and self.edge[i].y==b) or (self.edge[i].x==b and self.edge[i].y==a)):
        return i
    
    self.edge[self.nbEdges].set(a,b)
    self.nbEdges += 1
    return i

  # rotates according to angle x&y
  def rotate(self, angx, angy):
    zCamera = 100 # distance from cube to the eye of the camera
    Ox=24.5
    Oy=8.5   # position (x,y) of the frame center
    focal     = 30 # Focal of the camera
    i = 0 
    j = 0
    a = 0 
    b = 0
    c = 0
    cx=math.cos(angx);
    sx=math.sin(angx);
    cy=math.cos(angy);
    sy=math.sin(angy);

    self.m00=cy;   
    self.m01=0;  
    self.m02=-sy;
    self.m10=sx*sy;
    self.m11=cx; 
    self.m12=sx*cy;
    self.m20=cx*sy;
    self.m21=-sx;
    self.m22=cx*cy;

    for i in range(8):
      self.aligned[i].x = self.m00*self.local[i].x + self.m01*self.local[i].y + self.m02*self.local[i].z
      self.aligned[i].y = self.m10*self.local[i].x + self.m11*self.local[i].y + self.m12*self.local[i].z
      self.aligned[i].z = self.m20*self.local[i].x + self.m21*self.local[i].y + self.m22*self.local[i].z + zCamera
      self.screen[i].x  = math.floor((Ox+focal*self.aligned[i].x/self.aligned[i].z))
      self.screen[i].y  = math.floor((Oy-focal*self.aligned[i].y/self.aligned[i].z))	

    for i in range(12):
      self.edge[i].visible = False

    pa = Point()
    pb = Point()
    pc = Point()
    for i in range(6):
      pa = self.screen[ self.face[i].sommets[0] ]
      pb = self.screen[ self.face[i].sommets[1]	]
      pc = self.screen[ self.face[i].sommets[2] ]	

      back = ((pb.x-pa.x)*(pc.y-pa.y)-(pb.y-pa.y)*(pc.x-pa.x))<0
      if not back:
        for j in range(4):
          self.edge[self.face[i].ed[j]].visible = True

  # Draw the cube using the line method !
  def draw(self, lm=ledmat.LedMat()):
    ld = ledmat.LedMatDraw(lm)
    # Backface
    for i in range(12):
      e = self.edge[i];
      if(not e.visible):
        ld.line(
          int(self.screen[e.x].x), int(self.screen[e.x].y),
          int(self.screen[e.y].x), int(self.screen[e.y].y))

    # Frontface
    for i in range(12):
      e = self.edge[i];
      if(e.visible):
        ld.line(
          int(self.screen[e.x].x), int(self.screen[e.x].y),
          int(self.screen[e.y].x), int(self.screen[e.y].y))

