# -*- coding: utf-8 -*-
from pyIbex import *
import cv2
import numpy as np
import math
import time

#Position of pixel in the origin:
i0 =450
j0 =350
echellePixel=1.1693 # km/pixel:

#Euler method:
def euler(x,y,v,phi,u,dt):
  
        #Equations of movment:
        xp=v*math.cos(phi)
        yp=v*math.sin(phi)
        vp=u[0]-0.1*v
        phip=u[1]
        
        #Application of Euler:
        x=x+dt*xp
        y=y+dt*yp
        v=v+dt*vp
        phi=phi+dt*phip
        
        return x,y,v,phi
        
        

def computeTrail(imgBin):
    
    kernelTrail = np.ones((3,3),np.uint8)
    ImageTrail=cv2.erode(imgBin,kernelTrail,iterations=3)
    
    return ImageTrail
        
if __name__ == '__main__':
    

#Initialization of the variables:
  
    r=65*echellePixel  #Radius
    epsilon=echellePixel #Precision
    dt=1   #Step
    
    
    #Robots position:
    m=[[Interval(100).inflate(2.5) , Interval(45).inflate(2.5)],
        [Interval(100).inflate(2.5), Interval(-50).inflate(2.5)],
            [Interval(100).inflate(2.5), Interval(-150).inflate(2.5)]]

    #Robots yaw:
    phi=[math.pi,math.pi,math.pi]
    #Robots speed (less than 5m/sec):
    v=[15,15,15]
    #u1 and u2 belong to the interval [-1,1]:
    u=[[1,0],[1,0],[1,0]]
   
 
 
 
#Read the image of the Gascogne Golf in the gray color:
    img = cv2.imread('Gascogne2.png',0)
    print(img.shape)
#Binarisation of the image:
    retval, img_bin = cv2.threshold(img, 254, 1, cv2.THRESH_BINARY_INV)
#Calcul of the integral of the image:
    imgIntegral = cv2.integral(img_bin)
    
    #Dimension of the image:
    j_max,i_max=imgIntegral.shape
    imgIntegral=imgIntegral[0:j_max-1,0:i_max-1]
    j_max,i_max=imgIntegral.shape
    print(j_max,i_max)   
    image = np.asarray(img_bin).tolist(); 
   #Area of calcul:
    X0=IntervalVector([[-i0*echellePixel, (-i0+i_max-1)*echellePixel], [(j0-j_max+1)*echellePixel, j0*echellePixel]]) 

    
#Creation of an object of the class PavageGascogne:
    
    clsivia = pyIbex.clSIVIA(image)
    clsivia.setRecord("test_video.avi",1.0,True)
    #Loop for the draw:
    for t in range(10):
      time1=time.time()
      #Creation of an object of the class IncertitudeRobots:
      

      
      #SIVIA:
      lbox = clsivia.imSIVIA(X0,m,r**2,echellePixel,i0,j0,epsilon,True)
      print(len(lbox[0]),len(lbox[1]),len(lbox[2]),len(lbox[3]))
      print(time.time()-time1)
      
      #Robot 1
      m[0][0],m[0][1],v[0],phi[0]=euler(m[0][0],m[0][1],v[0],phi[0],u[0],dt)
      #Robot 2:
      m[1][0],m[1][1],v[1],phi[1]=euler(m[1][0],m[1][1],v[1],phi[1],u[1],dt)
      #Robot 3:
      m[2][0],m[2][1],v[2],phi[2]=euler(m[2][0],m[2][1],v[2],phi[2],u[2],dt)



