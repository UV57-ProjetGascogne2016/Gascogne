import math
import time
import numpy as np
from pyIbex import *
from vibes import vibes

class Robot:
    def __init__(self, x, y, V, theta, dt):
        #Etats
        self.X = np.array([x,y])
        self.V = V
        self.theta = theta
    
        #Objectifs
        self.Vhat = 0
        self.Xhat = np.array([0,0])
        
        #Commande
        self.u = np.array([0,0])
        self.kp = np.array([1,20])
        self.dt = dt
        
    def setObjectifs(self, X, V):
        self.Vhat = V
        self.Xhat = X
        return True
        
    def calculConsigne(self):
        w = self.Vhat - 2*(self.X - self.Xhat)
        vbar = np.linalg.norm(w)
        thetabar = math.atan2(w[1],w[0])
        return vbar, thetabar
        
    def controlCommande(self, alpha = 0.1):
        vbar, thetabar = self.calculConsigne()
        
        u_v = self.kp[0]*(vbar-self.V)
        u_theta = self.kp[1]*2*math.atan(math.tan(0.5*(thetabar-self.theta)))
        
        self.u = np.array([u_v,u_theta])
  
        return True
        
    def subMove(self):
        xp = self.V*math.cos(self.theta)
        yp = self.V*math.sin(self.theta)
        Vp = self.u[0] -0.1*self.V
        thetap = self.u[1]

         
        self.X[0] = self.X[0] + self.dt *xp
        self.X[1] = self.X[1] + self.dt *yp
        self.V = self.V + self.dt *Vp
        self.theta = self.theta + self.dt *thetap
    
        return True
        
    def getVandPsi(self):
        return True
    
def frange(start, stop, step):
    # range for floats
    i = start
    while i < stop:
        yield i
        i += step

if __name__ == '__main__':
    
    # simulation
    h=0.1
    
    rob1 = Robot(0.1,0.1,1,1,h)
    listRobots = [rob1]

    vibes.beginDrawing()
    vibes.clearFigure()  
    X0 = IntervalVector([[-10, 10], [-10, 10]])
    for t in frange(0,100,h):
        time1 = time.time()
        for rob in listRobots:
            #Draw circle of position
            # print(rob.X[0])
            # print(rob.X[1])
            vibes.drawCircle(rob.X[0], rob.X[1], 0.2, '[k]')
            #Determine next point
            nextX = np.array([10*math.cos(t),10*math.sin(t)])
            vibes.drawCircle(nextX[0],nextX[1], 0.1, 'r')
            nextV = np.array([-10*math.sin(t),10*math.cos(t)])
            print(nextX)
            print(nextV)
            rob.setObjectifs(nextX, nextV)
            
            #Calculate the new command
            rob.controlCommande()
            
            #Move the robot
            rob.subMove()
            time.sleep(0.1)

    
    vibes.endDrawing()