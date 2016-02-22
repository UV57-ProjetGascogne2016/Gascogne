# -*- coding: utf-8 -*-
"""
Created on Fri Feb 12 08:50:11 2016

Class IncertitudeRobots
Computes the secure zone around a robot with incertitude.
"""

from pyIbex import *
from SIVIA import *
import numpy as np
import math

class IncertitudeRobots:
  
  def __init__(self, m,rang2):
    self.l_m = m
    self.Bp = IntervalVector(len(m), [0,rang2])
      
  def testA(self, X,m):
    Xm = max(Interval(0), sign( (X[0]-m[0].ub())*(X[0]-m[0].lb()))) * \
              min(sqr(X[0]-m[0].lb()),sqr(X[0]-m[0].ub()) ) \
            + max(Interval(0), sign( (X[1]-m[1].ub())*(X[1]-m[1].lb()))) \
              * min(sqr(X[1]-m[1].lb()),sqr(X[1]-m[1].ub()) ) 

    Xp = max(sqr(X[0]-m[0].lb()),sqr(X[0]-m[0].ub())) + \
                  max(sqr(X[1]-m[1].lb()),sqr(X[1]-m[1].ub()))
      
    Xub = Xm | Xp
    if self.Bp[0].is_disjoint(Xub):
      return IBOOL.OUT
    elif Xub.is_subset(self.Bp[0]):
      return IBOOL.IN
    else:
      b1 = ( Xm - self.Bp[0].ub()).is_subset(Interval(-1000, 0))  
      B2 = ( self.Bp[0].ub() - Xp) 
      incl = False
      if  B2.ub() < 0:
          if (b1 ):
            return IBOOL.MAYBE
          else:
            return IBOOL.OUT
      return IBOOL.UNK

  def test(self, X):
    res = []
    for i,m in enumerate(self.l_m):
      res.append(self.testA(X,m))
    if IBOOL.IN in res:
      return IBOOL.IN
    if IBOOL.UNK in res:
      return IBOOL.UNK
    if IBOOL.MAYBE in res:
      return IBOOL.MAYBE
    test = True
    for r in res:
      test &= (r == IBOOL.OUT)
    if test:
      return IBOOL.OUT
    return IBOOL.UNK