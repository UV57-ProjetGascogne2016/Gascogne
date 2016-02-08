# sivia_calc.py
"""
sivia_calc calculate covert region from position of boxes
"""
# sivia_calc.py
from pyIbex import *

def getInBoxes(region,gps_pos,rang2,epsilon=1)
    return pyIbex.SIVIAtest(region,gps_pos,rang2,epsilon,True)

def getAllBoxes(region,gps_pos,rang2,epsilon=1,efficient=True)
    return pyIbex.fSIVIAtest(region,gps_pos,rang2,epsilon,efficient)
