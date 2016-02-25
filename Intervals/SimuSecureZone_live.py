# -*- coding: utf-8 -*-
from pyIbex import *
from simulationControl import *
import cv2
import numpy as np
import math
import pickle
import time
  
# Index of the frame origine pixel
i0 =410
j0 =400
echellePixel=1.1693 # km/pixel
radius=40 # Detection radius in km
PositionIncertitude = 1 # +- in km
epsilon=5 # SIVIA precision in km
enemySpeed = 100 # in m/s

if __name__ == '__main__':
    # Initialisation Simulation
    nbsRobots = 10
    simu = SimulationControl(nbsRobots,30000,1,500,200)
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
    imgIntegral=imgIntegral[0:j_max-1,0:i_max-1]
    j_max,i_max=imgIntegral.shape
    print(j_max,i_max)   
    image = np.asarray(img_bin).tolist();

    #Area of calcul / First box for SIVIA algorithm
    X0=IntervalVector([[-i0*echellePixel, (-i0+i_max-1)*echellePixel], [(j0-j_max+1)*echellePixel, j0*echellePixel]]) 

    imgOut=np.zeros((j_max, i_max),dtype="uint8")

    accPixelErode = 0 # accumulator of pixel to erode
    #Solver creation
    clsivia = pyIbex.clSIVIA(image,1,2,40)
    clsivia.setRecord("test_video.avi",25.0,True)
    clsivia.setScreening(True)
    #Loop
    for t in np.arange(0,simu.Tf,simu.dt):
        print("Time :",t," / ", simu.Tf/simu.dt)
        time1=time.time()
        t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, positions, X ,cons = simu.simuOneStep(t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f,X)
        
        for r in range(nbsRobots):
            m[r][0] = Interval(positions[r][1]).inflate(PositionIncertitude) #x
            m[r][1] = Interval(positions[r][2]).inflate(PositionIncertitude) #y

        accPixelErode = enemySpeed*simu.dt/(1000*echellePixel) + accPixelErode
        if (accPixelErode > 1):
            clsivia.setErode(True)
            print ("erode")
            accPixelErode = accPixelErode - 1
        else:
            clsivia.setErode(True)
            print("acc erode :",accPixelErode)
        #SIVIA:
        clsivia.limSIVIA(X0,m,radius**2,echellePixel,i0,j0,epsilon,True)
        print(time.time()-time1)

        t_old = t
