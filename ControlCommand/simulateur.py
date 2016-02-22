import math
import time
import numpy as np
from pyIbex import *
from vibes import vibes
from regulateur import *

# # # DEBUT PARTIE SIMULATION : EULER # # #
def simule(robot, pwm, dt):
    
    u = pwmTOu(pwm)
    
    xp = robot.V*math.cos(robot.thetaReel)
    yp = robot.V*math.sin(robot.thetaReel)
    Vp = u[0] -1*robot.V
    thetap = u[1]

    X = np.array([0.0,0.0])
    V = 0.0
    theta = 0.0       

    X[0] = robot.X[0] + dt *xp #+ np.random.normal(scale=0.1)
    X[1] = robot.X[1] + dt *yp #+ np.random.normal(scale=0.1)
    V = robot.V + dt *Vp #+ np.random.normal(scale=0.1)
    theta = robot.thetaReel + dt *thetap #+ np.random.normal(scale=0.1)

    return X,V,theta
    
def pwmTOu(pwm):
    
    u = np.array([0.0,0.0])
    
    u[0] = math.atanh((pwm[0]-1800)/200)+1
    u[1] = math.atanh((pwm[1]-1500)/500)
    
    return u
# # # FIN PARTIE SIMULATION # # #