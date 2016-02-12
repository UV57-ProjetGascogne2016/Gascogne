# -*- coding: utf-8 -*-
""" SIVIA for Secure Zone Processing
"""
import pyIbex
from pyIbex import IntervalVector, LargestFirst
from collections import deque
from vibes import vibes
from enum import Enum

class IBOOL(Enum):
    IN = 0
    OUT = 1
    MAYBE = 2
    UNK = 3
    EMPTY = 4
    BORDER = 5


def SIVIA(X0, test, test3, test2, eps):  
  stack =  deque([IntervalVector(X0)])
  lf = LargestFirst(eps/2.0)
  k = 0
  boatBoxesNP = []
  while len(stack) > 0:
    X = stack.popleft()
    k = k+1
    t = test.test(X)
    t2 = test2.test(X)
    t3 = test3.test(X)
    

    # if (t == IBOOL.IN):
    #   vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[g]' )
    if (t==IBOOL.IN or t2 == IBOOL.IN or t3 == IBOOL.IN):
      
      i,j = test.toPixels(X[0].lb(),X[1].ub())
      i1,j1 = test.toPixels(X[0].ub(),X[1].ub())
      i2,j2 = test.toPixels(X[0].ub(),X[1].lb())
      i3,j3 = test.toPixels(X[0].lb(),X[1].lb())
      boatBoxesNP.append([[i,j],[i1,j1],[i2,j2],[i3,j3]])
      if (t==IBOOL.IN): # Gascogne
          vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[g]' )
      elif (t2 == IBOOL.IN): # Robots
          vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[r]' )
      elif (t3 == IBOOL.IN): # Trail
          vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[r]' )
    elif (t == IBOOL.OUT and t2 == IBOOL.OUT and t3 == IBOOL.OUT):
      vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[b]' )
    elif (t2 == IBOOL.MAYBE):
       vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[orange]' )
    else:
      try:
        if (X.max_diam() > eps):
          (X1, X2) = lf.bisect(X)
          stack.append(X1)
          stack.append(X2)
        else :
          vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[y]' )
      except Exception:
         print(type(lf),lf)      
        
  vibes.axisEqual()
  return boatBoxesNP


