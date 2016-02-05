""" Simple SIVIA
"""
from enum import Enum
class IBOOL(Enum):
    IN = 0
    OUT = 1
    MAYBE = 2
    UNK = 3
    EMPTY = 4
    BORDER = 5

import pyIbex
from pyIbex import IntervalVector, LargestFirst
from collections import deque
from vibes import vibes

def SIVIA(X0, test, eps):
  stack =  deque([IntervalVector(X0)])
  lf = LargestFirst(eps/2.0)
  k = 0
  while len(stack) > 0:
    X = stack.popleft()
    k = k+1
    t = test.test(X)

    if (t == IBOOL.IN):
      vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[r]' )
    elif (t == IBOOL.OUT):
      vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[b]' )
    elif (t == IBOOL.MAYBE):
      vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[orange]' )
    else:
      if (X.max_diam() > eps):
        (X1, X2) = lf.bisect(X)
        stack.append(X1)
        stack.append(X2)
      else :
        vibes.drawBox(X[0][0],X[0][1], X[1][0], X[1][1], '[y]' )
  vibes.axisEqual()


