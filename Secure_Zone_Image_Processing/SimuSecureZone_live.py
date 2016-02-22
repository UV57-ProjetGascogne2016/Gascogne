# -*- coding: utf-8 -*-
from pyIbex import *
from vibes import vibes
from SIVIA import *
from IncertitudeRobots import *
from ImageToBoxes import *
from simulationControl import *
import cv2
import numpy as np
import math
import pickle
  
# Index of the frame origine pixel
i0 =410
j0 =400
echellePixel=1.1693 # km/pixel
radius=40 # Detection radius in km
PositionIncertitude = 1 # +- in km
epsilon=5 # SIVIA precision in km
enemySpeed = 100 # in m/s
            

def computeTrail(imgBin, sizekernel, iterations):
    
    kernelTrail = np.ones((sizekernel,sizekernel),np.uint8) 
    ImageTrail=cv2.erode(imgBin,kernelTrail,iterations)
    
    return ImageTrail
        


if __name__ == '__main__':
    # Initialisation Simulation
    nbsRobots = 3
    simu = SimulationControl(nbsRobots,550,20)
    theta_i = 0 # initial angle of the ellipse
    l_i = 0.1 # initial major axis of the ellipse
    centre_i = np.array([[0],[0]]) # initial center of the ellipse
    k = 0 # start with position number 0
    tk = 0
    l_f,theta_f,centre_f = simu.param_ellipse(simu.objectifs,k) # ellipse parameter to reach

    x0 = np.array([[0],[0],[0],[1]])
    X = np.dot(x0,np.ones((1,simu.N))) # position of the robots

        
    #Initial Robots position matrix:     
    m=[]
    for i in range(nbsRobots):
        x=[Interval(0).inflate(0) , Interval(0).inflate(0)]
        m.append(x)
         
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
    
    accPixelErode = 0 # accumulator of pixel to erode
    
    #Loop
    for t in np.arange(0,simu.Tf,simu.dt):
        
        t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, positions, X ,cons = simu.simuOneStep(t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f,X)
        
        for r in range(nbsRobots):
            m[r][0] = Interval(positions[r][1]).inflate(PositionIncertitude) #x
            m[r][1] = Interval(positions[r][2]).inflate(PositionIncertitude) #y
        
        #SIVIA test for Robots secure area whith incertitude
        pdcRobots = IncertitudeRobots(m,radius**2)
    
        # SIVIA test for Trail
        accPixelErode = enemySpeed*simu.dt/(1000*echellePixel) + accPixelErode
        print(accPixelErode)
        if (accPixelErode > 1):
            iterations = 1
            accPixelErode = accPixelErode - iterations
        else:
            iterations = 0
        imgTrail = computeTrail(imgOut, 1, iterations)
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