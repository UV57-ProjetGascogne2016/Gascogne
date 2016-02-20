import math
import time
import numpy as np
from pyIbex import *
from vibes import vibes

#TODO
#Adapt regulator to always provide a positive u
#Allow to transform u into PWM
#Make a function to determine the heading with GPS coordinates
#Be able to read a file with the coordinates of each ellipsoides and spead at each point and if necessary coasts

class Robot:
    def __init__(self, x, y, V, theta, numero, couleur, numRobots):
        
        #Etats actuels
        self.X = np.array([x,y])
        self.V = V
        self.thetaReel = theta
        self.thetaMesure = 0
        #Etat précédent
        self.Xavant = np.array([x,y])
        self.Vavant = V
        self.thetaavant = theta
        
        self.numero = numero
        self.totalRobots = numRobots
        self.couleur = couleur
    
        #Objectifs
        self.Vhat = 0.0
        self.Xhat = np.array([0,0])
        
        #Commande
        self.kp = np.array([6,15])
       
    def calculCap(self):
        theta = math.atan2(self.X[1] - self.Xavant[1], self.X[0] - self.Xavant[0])
        return theta
    
    def distanceCote(self):
        cotes = np.array([[-4.4,47.8],[-3.8,47.8],[-2.3,47.2],[-2,46.6],[-1.3,44.4],[-2.0,43.3],[-3.6,43.5],[-2.8,43.4]])
        distances = np.zeros([len(cotes)-1,2]) #to have same array as cotes
        for i in range(0,len(cotes)-1):
            distances[i] = dist(cotes[i],cotes[i+1],self.X)
        return distances   
        
    def setObjectifs(self, X, V):
        self.Vhat = V
        self.Xhat = X
        return True
        
    def calculConsigne(self):
        distances = self.distanceCote()
        w = self.Vhat - 2*(self.X - self.Xhat)
        # Repulsive fiel from the coasts
        # for dist in distances:
        #     w = w - 0.01*dist/(np.linalg.norm(dist)**3)
        vbar = np.linalg.norm(w)
        thetabar = math.atan2(w[1],w[0])
        return vbar, thetabar
        
    def controlCommande(self, alpha = 0.1):
        vbar, thetabar = self.calculConsigne()
        
        u_v = self.kp[0]*(vbar-self.V)+ np.random.normal(scale=0.1)
        u_theta = self.kp[1]*2*math.atan(math.tan(0.5*(thetabar-self.thetaMesure)))+ np.random.normal(scale=0.1)
        
        # print("u_v :", u_v)
        # print("u_theta :", u_theta)
        
        #Saturation acceleration
        if(u_v>1):
            u_v = 1
        elif(u_v<0):
            u_v = 0
            
        #Saturation en vitesse angulaire
        if(u_theta>10):
             u_theta = 10
        elif(u_theta<-10):
            u_theta = -10
        # Peut être remplacée par une saturation par la fonction tanh
        
        u = np.array([u_v,u_theta])
  
        return u
        
    def regulate(self,nextX, nextV):

        self.setObjectifs(nextX, nextV)
        
        #Calculate the new command
        u = self.controlCommande()
            
        return uTOpwm(u)
        
    def regulateOLD(self,t):
               
        #Determine next point
        delta = 2*self.numero*math.pi/self.totalRobots
        nextX, nextV = traj(t,delta)
        #vibes.drawCircle(nextX[0],nextX[1], 0.005, 'r')
        self.setObjectifs(nextX, nextV)
        
        #Calculate the new command
        u = self.controlCommande()
            
        return uTOpwm(u)
        
    def setNewState(self,X,V):
        #Sauvegarde de l'état précédent
        self.Xavant = self.X
        self.Vavant = self.V
        
        #Nouvel état
        self.X = X
        self.V = V
        
        #Determine le cap actuel du robot par différence finie
        self.thetaMesure = self.calculCap()
        # print("ThetaMesure = ",self.thetaMesure)
        
        return True
        
def uTOpwm(u):
    
    pwm = np.array([0.0,0.0])
    
    pwm[0] = 1600 + 100 * math.tanh(u[0]-1)
    
    pwm[1] = 1500 + 200 * math.tanh(u[1])
    
    return pwm

def dist(A, B, C): # C is the point
    AB = B-A
    AC = C-A
    BC = C-B
    
    normAB = np.linalg.norm(AB)

    u =  np.linalg.norm(AC*AB/normAB)

    if u > 1:
        u = 1
        return BC
    elif u < 0:
        return AC
    else:
        res = np.cross(AC,AB)/normAB
        return res
    
    
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
    recenter = np.array([-2,44.6])
    w = np.dot(T,c)+recenter
    dw = np.dot(dT,c) + np.dot(T,dc)

    return w, dw