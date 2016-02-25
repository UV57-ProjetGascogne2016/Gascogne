# -*- coding: utf-8 -*-
from pyIbex import *
from simulationControl import *
import cv2
import numpy as np
import math
import pickle
import time
import Client
  

SizeH = 500*2
SizeW = 600*2
# Ind5x of the frame origine pixel
i0 =100
j0 =SizeH - 20
echellePixel=0.05 # m/pixel
radius=4 # Detection radius in m
PositionIncertitude = 1 # +- in m
epsilon=0.1 # SIVIA precision in km
enemySpeed = 0.1 # in m/s


def toPixels(x, y):
        r"""get the value in the matrix imgIntegral at the position x, y in the interval world
        """
        i = round(x/echellePixel) + i0
        j = -round(y/echellePixel) + j0
        return (i, j)
        

if __name__ == '__main__':
    

    
         
    # SIVIA test for Gascogne Golf
    img_bin = np.zeros((SizeH,SizeW),np.uint8) #Binarisation of the image
    imgIntegral = cv2.integral(img_bin) #Calcul of the integral of the image
    j_max,i_max=imgIntegral.shape #Dimension of the image   
    imgIntegral=imgIntegral[0:j_max-1,0:i_max-1]
    j_max,i_max=imgIntegral.shape
    print(j_max,i_max)   
    NorthOrigin =5363846.097328593; 
    EstOrigin = 391039.7802301956;
    Point1 = toPixels(0,0)
    Point2 = toPixels(40.863285,4.2179)
    Point3 = toPixels(37.3211,39.4222)
    Point4 = toPixels(-3.5161,36.5381)
    cv2.line(img_bin,Point1, Point2, (1,1,1))
    cv2.line(img_bin, Point2, Point3, (1,1,1))
    cv2.line(img_bin, Point3, Point4, (1,1,1))
    image = np.asarray(img_bin).tolist();
    print("waiting connexion...")
    client = Client.Client('169.254.178.135',7000)
    print("Connected !!")
    #Area of calcul / First box for SIVIA algorithm
    X0=IntervalVector([[-i0*echellePixel, (-i0+i_max-1)*echellePixel], [(j0-j_max+1)*echellePixel, j0*echellePixel]]) 

    imgOut=np.zeros((j_max, i_max),dtype="uint8")
    
    accPixelErode = 0 # accumulator of pixel to erode
    #Solver creation
    clsivia = pyIbex.clSIVIA(image,1,2,1)#mask,size kernel (2*i+1),0 1 2 type of erosion (rectangle,cross,ellipse),iteration erosion
    clsivia.setRecord("test_video_via_server_0.avi",25.0,True)#name,fps,draw rectangle
    clsivia.setScreening(True)
    
    #Loop
    t_old = time.time()
    counter = time.time()
    while (1):
        #request server
        t = time.time()
        positions = client.getData()
        if positions==[]:
           break
        print(positions)
        m = []
        for i in range(len(positions)):
            if not (positions[i][0]==0 and positions[i][1]==0 and positions[i][2]==0):
                m.append([Interval(positions[i][1]-EstOrigin).inflate(PositionIncertitude),Interval(positions[i][2]-NorthOrigin).inflate(PositionIncertitude)])

        

        accPixelErode = enemySpeed*(t-t_old)/(echellePixel) + accPixelErode
        if (accPixelErode > 4):
            clsivia.setErode(True)
            print ("erode")
            accPixelErode = accPixelErode - 4
        else:
            clsivia.setErode(False)
            print("acc erode :",accPixelErode)
        #SIVIA:
        time1=time.time()
        clsivia.limSIVIA(X0,m,radius**2,echellePixel,i0,j0,epsilon,True)
        print("SIVIA : ",time.time()-time1,"All : ",time.time()-t)

        t_old = t
    client.close()
