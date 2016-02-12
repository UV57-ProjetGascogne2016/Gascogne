# -*- coding: utf-8 -*-
from pyIbex import *
from vibes import vibes
from SIVIA import *
from IncertitudeRobots import *
from ImageToBoxes import *
import cv2
import numpy as np
import math
import pickle
  
# Index of the frame origine pixel
i0 =450
j0 =350
echellePixel=1.1693 # km/pixel
r=10*echellePixel  # Detection radius
epsilon=echellePixel*5 # SIVIA precision
dt=1   # Euler time step
enemySpeed = 2

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
        
        

def computeTrail(imgBin, sizekernel):
    
    kernelTrail = np.ones((sizekernel,sizekernel),np.uint8)
    ImageTrail=cv2.erode(imgBin,kernelTrail,iterations=3)
    
    return ImageTrail
        


if __name__ == '__main__':

    # read a log file with the robots position
    with open('log', 'rb') as fichier:
        mon_depickler = pickle.Unpickler(fichier)
        position = mon_depickler.load()

    #Initial Robots position:
    m=[[Interval(0).inflate(0) , Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)],
        [Interval(0).inflate(0), Interval(0).inflate(0)]]

    
    
#    #Initial Robots position:
#    m=[[Interval(100).inflate(2.5) , Interval(45).inflate(2.5)],
#        [Interval(100).inflate(2.5), Interval(-50).inflate(2.5)],
#        [Interval(100).inflate(2.5), Interval(-150).inflate(2.5)]]
#
#    #Robots yaw:
#    phi=[math.pi,math.pi,math.pi]
#    #Robots speed (less than 5m/sec):
#    v=[15,15,15]
#    #u1 and u2 belong to the interval [-1,1]:
#    u=[[1,0],[1,0],[1,0]]
#   
 
    # Pavage of th Gascogne Golf 
    img = cv2.imread('Gascogne2.png',0) #Read the image of the Gascogne Golf in the gray color
    retval, img_bin = cv2.threshold(img, 254, 1, cv2.THRESH_BINARY_INV) #Binarisation of the image
    imgIntegral = cv2.integral(img_bin) #Calcul of the integral of the image
    j_max,i_max=imgIntegral.shape #Dimension of the image
    pdcGascogne = ImageToBoxes(imgIntegral,i0,j0,echellePixel) #Creation of an object of the class PavageGascogne    
    
    #Area of calcul:
    X0=IntervalVector([[-i0*echellePixel, (-i0+i_max-1)*echellePixel], [(j0-j_max+1)*echellePixel, j0*echellePixel]]) 

    imgOut=np.zeros((j_max, i_max),dtype="uint8")

    #Definition of the draw window:
    vibes.beginDrawing()
    vibes.newFigure('Robotique')
    vibes.setFigureProperties(dict(x=600, y=200, width=j_max, height=i_max))
    vibes.axisLimits(X0[0].lb(),X0[0].ub(),X0[1].lb(),X0[1].ub())
    
    #Loop
    for t in range(100):
        
        for r in range(10):
            m[r][0] = Interval(position[t][1][r]).inflate(2.5) #x
            m[r][1] = Interval(position[t][2][r]).inflate(2.5) #y
        
        
        pdcRobots = IncertitudeRobots(m,r**2) #Creation of an object of the class IncertitudeRobots
    
        # Trail
        imgTrail = computeTrail(imgOut, enemySpeed)
        imgIntegralTrail = cv2.integral(imgTrail)
        pdcTrail = ImageToBoxes(imgIntegralTrail,i0,j0,echellePixel)
          
        vibes.clearFigure('Robotique');
          
        boatBoxesNP = SIVIA(X0, pdcGascogne, pdcTrail, pdcRobots, epsilon)
          
        #Draw of the robots:
        for m_ in m:
            vibes.drawCircle(m_[0].mid(), m_[1].mid(), 0.2, '[k]')
        vibes.drawArrow([-15, -15], [-15, -10], 1, 'w[w]')
        vibes.drawArrow([-15, -15], [-10, -15], 1, 'w[w]')
    
          
        #Creation of the output image:
        imgT=np.zeros((j_max, i_max),dtype="uint8")
        imgOut=cv2.fillPoly(imgT,np.array( boatBoxesNP),(1,1,1))
    
          
#        #Modification with the new variables of the robots:
#        m[0][0],m[0][1],v[0],phi[0]=euler(m[0][0],m[0][1],v[0],phi[0],u[0],dt)
#        m[1][0],m[1][1],v[1],phi[1]=euler(m[1][0],m[1][1],v[1],phi[1],u[1],dt)
#        m[2][0],m[2][1],v[2],phi[2]=euler(m[2][0],m[2][1],v[2],phi[2],u[2],dt)

    vibes.endDrawing()