# -*- coding: utf-8 -*-
"""
Created on Fri Feb 12 08:56:20 2016

Class ImageToBoxes
Computes the pavage of an input image.
"""
from pyIbex import *
from SIVIA import *


class ImageToBoxes:
    
    #Position of pixel in the origin:

  
    def __init__(self, imgIntegral, i0, j0, echellePixel):
        self.l_imgIntegral = imgIntegral
        self.i0 = i0
        self.j0 = j0
        self.echellePixel = echellePixel
        
    def toPixels(self, x, y):
        r"""get the index (i,j) of the pixel coresponding to the position (x,y) in the real world
        """  
        i = round(x/self.echellePixel) + self.i0
        j = -round(y/self.echellePixel) + self.j0
        return i, j
        
    #Method for calculate the sum and the size of the box in the integral image:
    def calculatBox(self,X):
        #Calcul of the points A,B,C and D:
        Ai, Aj = self.toPixels(X[0].lb(),X[1].ub())
        Ci, Cj = self.toPixels(X[0].ub(),X[1].lb())
        Bi, Bj = self.toPixels(X[0].ub(),X[1].ub())
        Di, Dj = self.toPixels(X[0].lb(),X[1].lb())
        
        #Calcul of the sum:
        num = self.l_imgIntegral[Aj][Ai] + self.l_imgIntegral[Cj][Ci] - self.l_imgIntegral[Bj][Bi] - self.l_imgIntegral[Dj][Di]  # Sum=A + C - B - D  
        
        #Calcul of the size of the box:
        sizeBox = (Bi - Ai) * (Dj - Aj)
        
        return num, sizeBox

    #Method which create the pavage with SIVIA:
    def test(self, X):
        r"""Inclusion test:
        """
        try:
            NumPixels, sizeBox = self.calculatBox(X) 
        except:   
            return IBOOL.UNK
                    
        if (NumPixels == 0):
            return IBOOL.OUT
        elif (NumPixels == sizeBox):
            return IBOOL.IN
        else:
            return IBOOL.UNK