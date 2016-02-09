# -*- coding: utf-8 -*-
from pyIbex import *
from vibes import vibes
import cv2
import numpy as np
import math
import time

#Position of pixel in the origin:
i0 = 0
j0 = 0
echellePixel=1 # pixel/km:

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
            
    
def listBoxToDraw(boxes):
    drawable=[]
    for X in boxes:
       drawable.append((X[0][0],X[0][1], X[1][0], X[1][1]))
    return drawable
###################################################################################################################
##########################                           Main                                ##########################   
###################################################################################################################


if __name__ == '__main__':
    

#Initialization of the variables:
  
    r=30  #Radius
    epsilon=5*echellePixel #Precision
    dt=0.7   #Step
    
    
    #Robots position:
    m=[[Interval(400).inflate(2.5) , Interval(-300).inflate(2.5)],
        [Interval(450).inflate(2.5), Interval(-400).inflate(2.5)],
            [Interval(500).inflate(2.5), Interval(-350).inflate(2.5)]]

    #Robots yaw:
    phi=[math.pi,0,-math.pi/4]
    #Robots speed (less than 5m/sec):
    v=[4,4,4]
    #u1 and u2 belong to the interval [-1,1]:
    u=[[1,0],[-1,1],[0,-1]]
   
 
 
 
#Read the image of the Gascogne Golf in the gray color:
    img = cv2.imread('Gascogne2.png',0)
#Binarisation of the image:
    retval, img_bin = cv2.threshold(img, 254, 1, cv2.THRESH_BINARY_INV)
#Calcul of the integral of the image:
    imgIntegral = cv2.integral(img_bin)
    image = np.asarray(imgIntegral).tolist(); 
    #Dimension of the image:
    j_max,i_max=imgIntegral .shape
    
   #Area of calcul:
    X0=IntervalVector([[-i0*echellePixel, (-i0+i_max-1)*echellePixel], [(j0-j_max+1)*echellePixel, j0*echellePixel]]) 


#Definition of the draw window:
    vibes.beginDrawing()
    vibes.newFigure('Robotique')
    vibes.setFigureProperties(dict(x=0, y=10, width=500, height=500))
       
    clsivia = pyIbex.clSIVIA(image)
    #Loop for the draw:
    for t in range(1):
        time1 = time.time()
        #Creation of an object of the class IncertitudeRobots:
        
        #Clear the figure:
        vibes.clearFigure();
        
        #SIVIA:
        #lbox = pyIbex.imSIVIAtest(X0,m,r**2,image,echellePixel,i0,j0,0.5*epsilon,True)
        lbox = clsivia.greet(X0,m,r**2,echellePixel,i0,j0,epsilon,True)
        print(len(image),len(image[0]))
        print(len(lbox[0]),len(lbox[1]),len(lbox[2]),len(lbox[3]))
        print(time.time()-time1)
        
        vibes.drawBoxesUnion(listBoxToDraw(lbox[0]),'k[r]')
        vibes.drawBoxesUnion(listBoxToDraw(lbox[1]),'k[b]')
        vibes.drawBoxesUnion(listBoxToDraw(lbox[2]),'k[orange]')
        vibes.drawBoxesUnion(listBoxToDraw(lbox[3]),'k[y]')
        time.sleep(3)
        #Draw of the robots:
        for m_ in m:
            vibes.drawCircle(m_[0].mid(), m_[1].mid(), 0.2, '[k]')
        vibes.drawArrow([-15, -15], [-15, -10], 1, 'w[w]')
        vibes.drawArrow([-15, -15], [-10, -15], 1, 'w[w]')
        
        
        #Modification with the new variables of the robots:
        
        #Robot 1
        m[0][0],m[0][1],v[0],phi[0]=euler(m[0][0],m[0][1],v[0],phi[0],u[0],dt)
        #Robot 2:
        m[1][0],m[1][1],v[1],phi[1]=euler(m[1][0],m[1][1],v[1],phi[1],u[1],dt)
        #Robot 3:
        m[2][0],m[2][1],v[2],phi[2]=euler(m[2][0],m[2][1],v[2],phi[2],u[2],dt)
        

    vibes.endDrawing()
