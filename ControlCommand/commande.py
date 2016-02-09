import math
import time
import numpy as np
from pyIbex import *
from vibes import vibes

class Robot:
    def __init__(self, x, y, V, theta, dt, numero, couleur):
        #Etats
        self.X = np.array([x,y])
        self.V = V
        self.theta = theta
        
        self.numero = numero
        self.couleur = couleur
    
        #Objectifs
        self.Vhat = 0
        self.Xhat = np.array([0,0])
        
        #Commande
        self.u = np.array([0,0])
        self.kp = np.array([6,20])
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
        
        u_v = self.kp[0]*(vbar-self.V)+ np.random.normal(scale=0.1)
        u_theta = self.kp[1]*2*math.atan(math.tan(0.5*(thetabar-self.theta)))+ np.random.normal(scale=0.1)
        
        self.u = np.array([u_v,u_theta])
  
        return True
        
    def subMove(self):
        xp = self.V*math.cos(self.theta)
        yp = self.V*math.sin(self.theta)
        Vp = self.u[0] -0.1*self.V
        thetap = self.u[1]

         
        self.X[0] = self.X[0] + self.dt *xp #+ np.random.normal(scale=0.1)
        self.X[1] = self.X[1] + self.dt *yp #+ np.random.normal(scale=0.1)
        self.V = self.V + self.dt *Vp #+ np.random.normal(scale=0.1)
        self.theta = self.theta + self.dt *thetap #+ np.random.normal(scale=0.1)
    
        return True
        
    def getVandPsi(self):
        return True
    
def frange(start, stop, step):
    # range for floats
    i = start
    while i < stop:
        yield i
        i += step
        
def traj(t,delta):
    
    # paramètres
    
    f = 0.5
    
    theta = 0
    dtheta = 0
    
    # Déformation grand axe et petit axe
    D = np.array([[0.25, 0], [0, 0.5+0.005*t]])
    dD = np.array([[0, 0], [0, 0.005]])
    
    # Rotation de l'ellipse
    R = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    dR = np.array([[-dtheta*math.sin(theta), -dtheta*math.cos(theta)],[dtheta*math.cos(theta), -dtheta*math.sin(theta)]])
    
    # Transformation (rotation + déformation)
    T = np.dot(R,D)
    dT = np.dot(dR, D) + np.dot(R,dD)
    
    # Translation de l'ellipse

    Transla = np.array([0,0.1*t])
    dTransla = np.array([0,0.1])
    
    #
    
    c = np.array([math.cos(f*t+delta), math.sin(f*t+delta)])+Transla
    dc = np.array([-f*math.sin(f*t+delta), f*math.cos(f*t+delta)])+dTransla
    
    #Recentrer l'ellipse au niveau du golf de gascogne
    recenter = np.array([-4.0,45])
    w = np.dot(T,c)+recenter
    dw = np.dot(dT,c) + np.dot(T,dc)

    return w, dw
    
if __name__ == '__main__':
    
    # np.array([-10*math.sin(t),10*math.cos(t)])
    # np.array([10*math.cos(t),10*math.sin(t)])
    # simulation
    h=0.05
    listRobots = []
    for i in range(0,4):
        listRobots.append(Robot(-4.0,45,0,0,h,i,'b'))

    vibes.beginDrawing()
    vibes.clearFigure() 
    vibes.drawLine([[-4.4,47.8],[-3.8,47.8],[-2.3,47.2],[-2,46.6],[-1.3,47.7],[-1.3,44.4],[-2.0,43.3],[-3.6,43.5],[-2.8,43.4]]) 
    for t in frange(0,2,h):
        #time.sleep(0.05)
        # vibes.clearFigure()
        time1 = time.time()
        for rob in listRobots:
            #Draw circle of position
            # print(rob.X[0])
            # print(rob.X[1])
            vibes.drawCircle(rob.X[0], rob.X[1], 0.02, rob.couleur)
            
            #Determine next point
            delta = 2*rob.numero*math.pi/len(listRobots)
            nextX, nextV = traj(t,delta)
            vibes.drawCircle(nextX[0],nextX[1], 0.01, 'r')
            
            # print(nextX)
            # print(nextV)
            rob.setObjectifs(nextX, nextV)
            
            #Calculate the new command
            rob.controlCommande()
            
            #Move the robot
            rob.subMove()


    
    vibes.endDrawing()