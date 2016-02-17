# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 09:43:06 2016

@author: alice
"""
# Control N robots on an ellipse that move throught the Bay of Biscay
# Output:
#   - for replay : a log file readable with pickle of the positions of N robots from t=0 to Tf
#   - for live simulation: TODO : send the positions of N robots at each iteration of t

import numpy as np
from vibes import *
import pickle
import time
import math

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
        self.kp = np.array([10,20])
       
    def calculCap(self):
        theta = math.atan2(self.X[1] - self.Xavant[1], self.X[0] - self.Xavant[0])
        return theta
        
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
        u_theta = self.kp[1]*2*math.atan(math.tan(0.5*(thetabar-self.thetaMesure)))+ np.random.normal(scale=0.1)
        
        # print("u_v :", u_v)        
        # print("u_theta :", u_theta)  
              
        #Saturation acceleration
        # if(u_v>10):
        #     u_v = 10
        # elif(u_v<0):
        #     u_v = 0
            
        #Saturation en vitesse angulaire
        # if(u_theta>10):
        #      u_theta = 10
        # elif(u_theta<-10):
        #     u_theta = -10
        # Peut être remplacée par une saturation par la fonction tanh
        
        u = np.array([u_v,u_theta])
  
        return u
        
    def regulate(self,t):
               
        #Determine next point
        delta = 2*self.numero*math.pi/self.totalRobots
        nextX, nextV = traj(t,delta)
        vibes.drawCircle(nextX[0],nextX[1], 0.005, 'r')
        self.setObjectifs(nextX, nextV)
        
        #Calculate the new command
        u = self.controlCommande()
            
        return uTOpwm(u)
        
    def regulateNEW(self,nextX, nextV):
               
        #Determine next point
        self.setObjectifs(np.array([nextX[0,0],nextX[1,0]]), np.linalg.norm(nextV))
        
        #Calculate the new command
        u = self.controlCommande()
            
        return u
        
    def setNewState(self,X,V, theta):
        #Sauvegarde de l'état précédent
        self.Xavant = self.X
        self.Vavant = self.V
        
        #Nouvel état
        self.X = X
        self.V = V
        
        #Determine le cap actuel du robot par différence finie
        self.thetaMesure = self.calculCap()
        self.thetaReel = theta
        
        return True

class SimulationControl:
    def __init__(self, N,temps_fin,tps_ellipse):
        
        self.N = N # number of boats
        self.dt = 0.01 # simulation step
        self.listRobots = [] # list of boats
        for i in range(0,N):
            self.listRobots.append(Robot(0.0,0.0,0.0,1.0,i,'b',N))
        self.Tf = temps_fin # end of simulation
        self.tps_ellipse=tps_ellipse # system speed (ellipse goal reached after tps_ellipse)
        
        # coast points (center at (45N,4W) )
        self.points =[[-31.5397, 311.1665],[15.7698, 311.1665],[134.0436, 244.4880],[157.6983, 177.8094],[212.8927,-66.6785],[157.6983,-188.9225],[94.6190,-177.8094],[31.5397,-166.6963],[-145.8710,-155.5832]]        
        self.objectifs=[[-31.5397, 301.1665],[5.769799999999955, 301.1665], [114.0436, 244.488], [137.69829999999993, 177.8094000000002],[165.2955, 55.56545], [192.8927, -66.67850000000004], [142.6456712727633, -175.3946918551941],[94.6190,-167.8094], [41.5397000000002, -156.6963],[-145.8710,-145.5832]]

    def xdot(self,x,u):
        theta=x[2]
        v=x[3]
        xdot=np.array([[v*np.cos(theta)], [v*np.sin(theta)], [u[0]], [u[1]]])
        return xdot
        
    def control(self,x,w,dw,ddw):
        v=0.25*(w - np.array([[x[0]],[x[1]]]))+1*(dw - np.array([[x[3]*np.cos(x[2])],[x[3]*np.sin(x[2])]]))+ddw    
        A=np.array([[-x[3]*np.sin(x[2]), np.cos(x[2])],[x[3]*np.cos(x[2]),np.sin(x[2])]])
        u=np.dot(np.linalg.inv(A),v)
        return u
    
    def Euler(self,robot, u):
        xp = robot.V*math.cos(robot.thetaReel)
        yp = robot.V*math.sin(robot.thetaReel)
        Vp = u[0] -1*robot.V
        thetap = u[1]
    
        X = np.array([0.0,0.0])
        V = 0.0
        theta = 0.0       
    
        X[0] = robot.X[0] + self.dt *xp #+ np.random.normal(scale=0.1)
        X[1] = robot.X[1] + self.dt *yp #+ np.random.normal(scale=0.1)
        V = robot.V + self.dt *Vp #+ np.random.normal(scale=0.1)
        theta = robot.thetaReel + self.dt *thetap #+ np.random.normal(scale=0.1)
    
        robot.setNewState(X,V,theta)
    
        return True
    
    def param_ellipse(self,p,k):
        # computation of ellipse parameters
        couple=[[5,6],[5,7],[5,8],[4,8],[3,8],[2,8],[1,9],[0,9]]
        i=couple[k][0]
        j=couple[k][1]
        l=np.sqrt((p[i][0]-p[j][0])**2+(p[i][1]-p[j][1])**2)/2
        theta = np.arctan((p[i][1]-p[j][1])/(p[i][0]-p[j][0]))
        c=np.array([[p[j][0]+(p[i][0]-p[j][0])/2],[p[j][1]+(p[i][1]-p[j][1])/2]])
        return l,theta,c
        
    def simuOneStep(self,t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f):
        toSend =np.ones((self.N,3)) # data to send on each loop
        f1 = 0.05
        if(k < 7): # transitional period
            if(tk >= self.tps_ellipse): # target ellipse reached, get next
                k = k+1
                tk = 0
                theta_i = theta_f
                l_i = l_f
                centre_i = centre_f
                l_f,theta_f,centre_f = self.param_ellipse(self.objectifs,k)
                
        # evolution of ellipse angle
        theta = theta_i + (theta_f - theta_i)*tk/self.tps_ellipse
        dtheta = (theta_f - theta_i)/self.tps_ellipse
        
        # evolution of ellipse center
        centre = centre_i+(centre_f - centre_i)*tk/self.tps_ellipse
        dcentre = (centre_f - centre_i)/self.tps_ellipse

        # evolution of ellipse major axis
        mAxis = l_i+(l_f-l_i)*tk/self.tps_ellipse
        dmAxis = (l_f-l_i)/self.tps_ellipse
        
        # evolution matrix of ellipse
        D = np.array([[ mAxis ,0],[0,18]])
        dD = np.array([[dmAxis ,0],[0,0]])
        
        R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        dR = dtheta*np.array([[-np.sin(theta), np.cos(theta)],[np.cos(theta), -np.sin(theta)]])
        
        T = np.dot(R,D)
        dT = np.dot(R,dD)+np.dot(dR,D)
            
        if(k==7 and tk > self.tps_ellipse): # permanent regime, ellipse is not moving
            
            theta = theta_f
            dtheta = 0
            
            centre = centre_f
            dcentre = np.array([[0],[0]])
            
            D = np.array([[l_f,0],[0,0.1]])
            dD = np.array([[0,0],[0,0]])
            
            R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta), np.cos(theta)]])
            dR = np.array([[0, 0],[0, 0]])
            
            T = np.dot(R,D)
            dT = np.dot(R,dD)+np.dot(dR,D)
        
        cons = []
        tk = tk + self.dt
        
        #for i in range(self.N): # computation for each robot
        for rob in self.listRobots:
            delta = 2*rob.numero*np.pi/self.N            

            c = np.array([[np.cos(f1*t+delta)], [np.sin(f1*t+delta)]])
            dc = np.array([[-f1*np.sin(f1*t+delta)],[f1*np.cos(f1*t+delta)]])
            ddc = np.array([[-f1*f1*np.cos(f1*t+delta)],[-f1*f1*np.sin(f1*t+delta)]])
            
            w = [centre[0],centre[1]]+np.dot(T,c)
            dw = [dcentre[0],dcentre[1]]+np.dot(T,dc)+np.dot(dT,c)
            #ddw = np.dot(T,ddc)
            cons.append(w)
            
            #u = self.control(X[:,i],w,dw,ddw)
            u = rob.regulateNEW(w,dw)
            
            #X[:,i] = X[:,i]+np.transpose(self.xdot(X[:,i],u)*self.dt)
            self.Euler(rob, u)
            
            # data to send ([time , Xposition, Yposition] for each robot)
            
            toSend[rob.numero,0] = t
            toSend[rob.numero,1] = rob.X[0]
            toSend[rob.numero,2] = rob.X[1]
            
        return t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, toSend, cons
           
        
    def simulation(self):
        theta_i = 0 # initial angle of the ellipse
        l_i = 0.1 # initial major axis of the ellipse
        centre_i = np.array([[0],[0]]) # initial center of the ellipse
        k = 0 # start with position number 0
        tk = 0
        l_f,theta_f,centre_f = self.param_ellipse(self.objectifs,k) # ellipse parameter to reach

        x0 = np.array([[0],[0],[0],[1]])
        X = np.dot(x0,np.ones((1,self.N))) # position of the robots
    
        
        toLog=[] # data to log at the end of the simulation
    

        # Figure parameters 
        vibes.beginDrawing()
        vibes.newFigure('Simulation Gascogne avec Potentiels')
        vibes.setFigureProperties({'x': 200, 'y': 100, 'width': 800, 'height': 800})
        vibes.drawLine(self.points) # drawing of the coast
        vibes.drawLine(self.objectifs,'green')  # drawing of ellipse target bounds 
        
        for t in np.arange(0,self.Tf,self.dt):
            
            t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, toSend, cons = self.simuOneStep(t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f)
            
            # draw new points on figure
            vibes.clearFigure()
            vibes.drawLine(self.points, 'black')
            vibes.drawLine(self.objectifs,'green')
            for rob in self.listRobots:
                vibes.drawCircle(rob.X[0],rob.X[1],5,'blue') # or use function drawAUV
                #vibes.drawAUV(rob.X[0], rob.X[1], 0.5, rob.thetaReel*180/math.pi, 'blue')
                vibes.drawCircle(cons[rob.numero][0][0],cons[rob.numero][1][0],2,'[red]') # consigne
            # time.sleep(0.1)
        vibes.endDrawing()
        
        return toLog
        
    def log(self,toLog):
        
        # save in a binary file log
        with open('log', 'wb') as fichier:
            mon_pickler = pickle.Pickler(fichier)
            mon_pickler.dump(toLog)        
        
if __name__ == '__main__':
    gascogne = SimulationControl(10,550,50)
    toLog = gascogne.simulation()
    gascogne.log(toLog)    
        
        

       
