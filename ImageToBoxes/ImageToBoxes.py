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

# position of the origine frame pixel
i0 = 500
j0 = 100
pixelScale = 10 # Km/pixel
epsilon = 10*pixelScale

class ClassTest:
    def __init__(self, imgIntegral):
        self.l_imgIntegral = imgIntegral
        
    def toPixels(self, x, y):
        r"""get the index (i,j) of the pixel coresponding to the position (x,y) in the real world
        """        
        i = round(x/pixelScale) + i0
        j = -round(y/pixelScale) + j0
        
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
        """
        try:
            NumPixels, sizeBox = self.calculatBox(X)       
            if (NumPixels == 0):
                return IBOOL.OUT
            elif (NumPixels == sizeBox):
                return IBOOL.IN
            else:
                return IBOOL.UNK
        except:
            return IBOOL.UNK

if __name__ == '__main__':
    img = cv2.imread('Gascogne.png',0)
    retval, img_bin = cv2.threshold(img, 254, 1, cv2.THRESH_BINARY_INV)
    imgIntegral = cv2.integral(img_bin)
    
    
#    cv2.namedWindow('img', cv2.WINDOW_AUTOSIZE )
#    cv2.imshow('img',img)
#    cv2.waitKey(0)

    j_max, i_max = imgIntegral.shape    
    X0 = IntervalVector([[-i0*pixelScale, (-i0+i_max-1)*pixelScale],[(j0-j_max+1)*pixelScale, j0*pixelScale]])
    pdc = ClassTest(imgIntegral)
    
    vibes.beginDrawing()
    vibes.newFigure('PavedImageSIVIA')
    vibes.setFigureProperties(dict(x=0, y=10, width=500, height=500))
    SIVIA(X0, pdc, epsilon)
    vibes.endDrawing()
    
    