# -*- coding: utf-8 -*-
from pyIbex import *
from simulationControl import *
import cv2
import numpy as np
import math
import pickle
import time
import Client
  
# Index of the frame origine pixel
i0 =410
j0 =400
echellePixel=1.1693 # km/pixel
radius=40 # Detection radius in km
PositionIncertitude = 1 # +- in km
epsilon=5 # SIVIA precision in km
enemySpeed = 100 # in m/s

if __name__ == '__main__':
    

    client = Client.Client('127.0.0.1',7000)
    
         
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
    clsivia = pyIbex.clSIVIA(image,1,2,3)
    clsivia.setRecord("test_video_via_server_0.avi",25.0,True)
    #Loop
    t_old = 0
    while (1):
        time1=time.time()
        #request server
        positions = client.getData()
        if positions==[]:
           break
        t = positions[0][0]
        m = []
        for i in range(len(positions)):
            m.append([Interval(positions[i][1]).inflate(PositionIncertitude),Interval(positions[i][2]).inflate(PositionIncertitude)])

        accPixelErode = enemySpeed*(t-t_old)/(1000*echellePixel) + accPixelErode
        if (accPixelErode > 1):
            clsivia.setErode(True)
            print ("erode")
            accPixelErode = accPixelErode - 1
        else:
            clsivia.setErode(False)
            print("acc erode :",accPixelErode)
        #SIVIA:
        clsivia.imSIVIA(X0,m,radius**2,echellePixel,i0,j0,epsilon,True)
        print(time.time()-time1)

        t_old = t
    client.close()
