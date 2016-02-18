# -*- coding: utf-8 -*-
from pyIbex import *
from simulationControl import *
import cv2
import numpy as np
import math
import pickle
import time
import Client
  
# Ind5x of the frame origine pixel
i0 =800
j0 =500
echellePixel=0.1 # m/pixel
radius=10 # Detection radius in m
PositionIncertitude = 1 # +- in m
epsilon=1 # SIVIA precision in km
enemySpeed = 0.1 # in m/s

if __name__ == '__main__':
    

    client = Client.Client('169.254.178.135',7000)
    
         
    # SIVIA test for Gascogne Golf
    #img = cv2.imread('Gascogne2.png',0) #Read the image of the Gascogne Golf in the gray color
    img_bin = np.zeros((1024,1980),np.uint8) #Binarisation of the image
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
    clsivia = pyIbex.clSIVIA(image,1,2,2)
    clsivia.setRecord("test_video_via_server_0.avi",25.0,True)
    #
    NorthOrigin =5363846.097328593; 
    EstOrigin = 391039.7802301956;
    #Loop
    t_old = time.time()
    counter = time.time()
    while (1):
        #request server
        positions = client.getData()
        if positions==[]:
           break
        t = time.time()
        m = []
        for i in range(len(positions)):
            if not (positions[i][0]==0 and positions[i][1]==0 and positions[i][2]==0):
                m.append([Interval(positions[i][1]-EstOrigin).inflate(PositionIncertitude),Interval(positions[i][2]-NorthOrigin).inflate(PositionIncertitude)])

        print(m,len(m))

        accPixelErode = enemySpeed*(t-t_old)/(echellePixel) + accPixelErode
        if (accPixelErode > 1):
            clsivia.setErode(True)
            print ("erode")
            accPixelErode = accPixelErode - 1
        else:
            clsivia.setErode(False)
            print("acc erode :",accPixelErode)
        #SIVIA:
        time1=time.time()
        clsivia.imSIVIA(X0,m,radius**2,echellePixel,i0,j0,epsilon,True)
        print("SIVIA : ",time.time()-time1,"All : ",time.time()-t)

        t_old = t
    client.close()
