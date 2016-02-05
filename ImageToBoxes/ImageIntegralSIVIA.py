# -*- coding: utf-8 -*-
"""
Created on Fri Feb  5 10:07:20 2016

@author: mael

Image integral
"""
from pyIbex import *
from vibes import vibes
from SIVIA import *
import cv2
import numpy as np

i0 = 0
j0 = 0

class ClassTest:
    def __init__(self, imgIntegral):
        self.l_imgIntegral = imgIntegral
        
    def toPixels(self, x, y):
        r"""get the value in the matrix imgIntegral at the position x, y in the interval world
        """
        i = round(x) + i0
        j = -round(y) + j0
        return i, j
        
    def calculatBox(self,X):

        Ai, Aj = self.toPixels(X[0].lb(),X[1].ub())
        Ci, Cj = self.toPixels(X[0].ub(),X[1].lb())
        Bi, Bj = self.toPixels(X[0].ub(),X[1].ub())
        Di, Dj = self.toPixels(X[0].lb(),X[1].lb())
        

        # A + C - B - D       
        num = self.l_imgIntegral[Aj][Ai] + self.l_imgIntegral[Cj][Ci] - self.l_imgIntegral[Bj][Bi] - self.l_imgIntegral[Dj][Di]
        
        sizeBox = (Bi - Ai) * (Dj - Aj)
        
        return num, sizeBox

    def test(self, X):
        r"""Inclusion test:
        
        inputs: -imgage Integral
                - Box
        -------
        """
#        if (X[0].ub() < 10) and (X[0].lb() > -20) and (X[1].ub() < 30) and (X[1].lb() > -40) :
#            return IBOOL.IN
#        else:
#            return IBOOL.UNK
        try:
            NumPixels, sizeBox = self.calculatBox(X)
            print('try ok')            
            if (NumPixels == 0):
                return IBOOL.OUT
            elif (NumPixels == sizeBox):
                return IBOOL.IN
            else:
                return IBOOL.UNK
        except:
            print('try fail')   
            return IBOOL.UNK

if __name__ == '__main__':
    img = cv2.imread('france.jpg',0)
    retval, img_bin = cv2.threshold(img, 254, 1, cv2.THRESH_BINARY_INV)
    imgIntegral = cv2.integral(img_bin)
    
    
    cv2.namedWindow('img', cv2.WINDOW_AUTOSIZE )
    cv2.imshow('img',img)

    
    
    X0 = IntervalVector(2, [0, 650])
    pdc = ClassTest(imgIntegral)
    
    vibes.beginDrawing()
    vibes.newFigure('ImageIntegralSIVIA')
    vibes.setFigureProperties(dict(x=0, y=10, width=500, height=500))
    SIVIA(X0, pdc, 10)
    vibes.endDrawing()
    
    cv2.waitKey(0)
    
    