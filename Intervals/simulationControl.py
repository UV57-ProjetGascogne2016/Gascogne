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

class SimulationControl:
    def __init__(self, N,temps_fin,tps_ellipse):
        
        self.N = N # number of boats
        self.Tf = temps_fin # end of simulation
        self.dt = 1 # simulation step
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
    
    
    def param_ellipse(self,p,k):
        # computation of ellipse parameters
        couple=[[5,6],[5,7],[5,8],[4,8],[3,8],[2,8],[1,9],[0,9]]
        i=couple[k][0]
        j=couple[k][1]
        l=np.sqrt((p[i][0]-p[j][0])**2+(p[i][1]-p[j][1])**2)/2
        theta = np.arctan((p[i][1]-p[j][1])/(p[i][0]-p[j][0]))
        c=np.array([[p[j][0]+(p[i][0]-p[j][0])/2],[p[j][1]+(p[i][1]-p[j][1])/2]])
        return l,theta,c
        
    def simuOneStep(self,t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f,X):
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
        
        for i in range(self.N): # computation for each robot
            delta = 2*i*np.pi/self.N            

            c = np.array([[np.cos(f1*t+delta)], [np.sin(f1*t+delta)]])
            dc = np.array([[-f1*np.sin(f1*t+delta)],[f1*np.cos(f1*t+delta)]])
            ddc = np.array([[-f1*f1*np.cos(f1*t+delta)],[-f1*f1*np.sin(f1*t+delta)]])
            
            w = [centre[0],centre[1]]+np.dot(T,c)
            dw = [dcentre[0],dcentre[1]]+np.dot(T,dc)+np.dot(dT,c)
            ddw = np.dot(T,ddc)
            cons.append(w)
            
            u = self.control(X[:,i],w,dw,ddw)
            X[:,i] = X[:,i]+np.transpose(self.xdot(X[:,i],u)*self.dt)
            
            # data to send ([time , Xposition, Yposition] for each robot)
            
            toSend[i,0] = t
            toSend[i,1] = X[0,i]
            toSend[i,2] = X[1,i]
            
        return t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, toSend, X, cons
           
        
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
        vibes.newFigure('Ellispe')
        vibes.setFigureProperties({'x': 200, 'y': 100, 'width': 800, 'height': 800})
        vibes.drawLine(self.points) # drawing of the coast
        vibes.drawLine(self.objectifs,'green')  # drawing of ellipse target bounds 
        
        for t in np.arange(0,self.Tf,self.dt):
            
            t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, toSend, X ,cons = self.simuOneStep(t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f,X)
            
            # draw new points on figure
            vibes.clearFigure()
            vibes.drawLine(self.points, 'black')
            vibes.drawLine(self.objectifs,'green')
            for i in range(self.N):
                vibes.drawCircle(X[0,i],X[1,i],5,'blue') # or use function drawAUV
                vibes.drawCircle(cons[i][0][0],cons[i][1][0],2,'[red]')
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
        
        

       
