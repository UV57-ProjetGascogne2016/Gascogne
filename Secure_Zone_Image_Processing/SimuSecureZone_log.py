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
i0 =410
j0 =400
echellePixel=1.1693 # km/pixel
r=10 # Detection radius in km
PositionIncertitude = 1 # +- in km
epsilon=5 # SIVIA precision in km
dt=1   # Euler time step in s
enemySpeed = 2 # in m/s
            

def computeTrail(imgBin, sizekernel):
    
    kernelTrail = np.ones((sizekernel,sizekernel),np.uint8) 
    ImageTrail=cv2.erode(imgBin,kernelTrail,iterations=3)
    
    return ImageTrail
        


if __name__ == '__main__':

    # read a log file with the robots position
    with open('log', 'rb') as fichier:
        mon_depickler = pickle.Unpickler(fichier)
        position = mon_depickler.load()
        
    #Initial Robots position matrix:    
    nbsRobots = len(position[0])
    print(nbsRobots)
    m=[]
    for i in range(nbsRobots):
        x=[Interval(0).inflate(0) , Interval(0).inflate(0)]
        m.append(x)
    t_old = m[0][0][0] 
 
    # SIVIA test for Gascogne Golf
    img = cv2.imread('Gascogne2.png',0) #Read the image of the Gascogne Golf in the gray color
    retval, img_bin = cv2.threshold(img, 254, 1, cv2.THRESH_BINARY_INV) #Binarisation of the image
    imgIntegral = cv2.integral(img_bin) #Calcul of the integral of the image
    j_max,i_max=imgIntegral.shape #Dimension of the image
    pdcGascogne = ImageToBoxes(imgIntegral,i0,j0,echellePixel) #Creation of an object of the class PavageGascogne    
    
    #Area of calcul / First box for SIVIA algorithm
    X0=IntervalVector([[-i0*echellePixel, (-i0+i_max-1)*echellePixel], [(j0-j_max+1)*echellePixel, j0*echellePixel]]) 

    imgOut=np.zeros((j_max, i_max),dtype="uint8")

    #Definition of the draw window:
    vibes.beginDrawing()
    vibes.newFigure('Robotique')
    vibes.setFigureProperties(dict(x=600, y=200, width=j_max, height=i_max))
    vibes.axisLimits(X0[0].lb(),X0[0].ub(),X0[1].lb(),X0[1].ub())
    
    #Loop
    for i in range(100):
        
        for r in range(nbsRobots):
            m[r][0] = Interval(position[i][r][1]).inflate(PositionIncertitude) #x
            m[r][1] = Interval(position[i][r][2]).inflate(PositionIncertitude) #y
        t = m[i][0][0]
        
        #SIVIA test for Robots secure area whith incertitude
        pdcRobots = IncertitudeRobots(m,r**2)
    
        # SIVIA test for Trail
        sizeKernelErosion = enemySpeed*(t-t_old)/(1000*echellePixel)
        print(sizeKernelErosion)
        imgTrail = computeTrail(imgOut, enemySpeed)
        imgIntegralTrail = cv2.integral(imgTrail)
        pdcTrail = ImageToBoxes(imgIntegralTrail,i0,j0,echellePixel)
          
        vibes.clearFigure('Robotique')
        boatBoxesNP = SIVIA(X0, pdcGascogne, pdcTrail, pdcRobots, epsilon)
          
        #Draw of the robots:
        for m_ in m:
            vibes.drawCircle(m_[0].mid(), m_[1].mid(), 0.2, '[k]')
        vibes.drawArrow([-15, -15], [-15, -10], 1, 'w[w]')
        vibes.drawArrow([-15, -15], [-10, -15], 1, 'w[w]')
    
          
        #Creation of the output image:
        imgT=np.zeros((j_max, i_max),dtype="uint8")
        imgOut=cv2.fillPoly(imgT,np.array( boatBoxesNP),(1,1,1))
    
        t_old = t
        

    vibes.endDrawing()