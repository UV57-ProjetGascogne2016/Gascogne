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

# MAIN INUTILE
if __name__ == '__main__':
    
    # # # DEBUT paramètres de la simulation # # #
    h=0.05
    listRobots = []
    for i in range(0,1):
        listRobots.append(Robot(-4.0,45,0,0,h,i,'b'))
    # # # FIN paramètres de la simulation # # #

    vibes.beginDrawing()
    vibes.clearFigure() 
    # Tracer le contour du golf de gascogne
    vibes.drawLine([[-4.4,47.8],[-3.8,47.8],[-2.3,47.2],[-2,46.6],[-1.3,44.4],[-2.0,43.3],[-3.6,43.5],[-2.8,43.4]])
    
    # # # DEBUT Déroulement de la simulation EN DUR # # # 
    for t in frange(0,100,h):
        time.sleep(0.1)
        #vibes.clearFigure()
        time1 = time.time()
        for rob in listRobots:
            #Draw circle of position
            # print(rob.X[0])
            # print(rob.X[1])
            vibes.drawAUV(rob.X[0], rob.X[1], 0.2, rob.theta*180/math.pi, rob.couleur)
            
            #Determine next point
            delta = 2*rob.numero*math.pi/len(listRobots)
            nextX, nextV = traj(t,delta)
            vibes.drawCircle(nextX[0],nextX[1], 0.005, 'r')
            
            # print(nextX)
            # print(nextV)
            rob.setObjectifs(nextX, nextV)
            
            #Calculate the new command
            rob.controlCommande()
            
            #Move the robot
            rob.subMove()

    vibes.endDrawing()
    # # # FIN Déroulement de la simulation EN DUR # # # 