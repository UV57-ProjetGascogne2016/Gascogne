import math
import time
import numpy as np
from pyIbex import *
from vibes import vibes
from regulateur import *
from simulateur import *

def frange(start, stop, step):
    # range for floats
    i = start
    while i < stop:
        yield i
        i += step

if __name__ == '__main__':
    
    # pour une durée T on simule
    # INITIALISATION : bateaux (positions, vitesse)
    T = 100
    h = 0.05
    nombreRobots = 1
    listRobots = []
    for i in range(0,nombreRobots):
        listRobots.append(Robot(-4.0,45.0,0.0,0.0,i,'b',nombreRobots))

    vibes.beginDrawing()
    vibes.clearFigure() 
    vibes.drawLine([[-4.4,47.8],[-3.8,47.8],[-2.3,47.2],[-2,46.6],[-1.3,44.4],[-2.0,43.3],[-3.6,43.5],[-2.8,43.4]])
    
    for t in frange(0,T,h):
        # DEROULEMENT pour la simulation:
        # u = Commande renvoyée par chaque régulateur de chaque bateau
        # Une itération de la simulation grâce à u
        # On change les positions des bateaux
        # On recommence
        for rob in listRobots:
            
            vibes.drawAUV(float(rob.X[0]), float(rob.X[1]), 0.2, rob.thetaReel*180/math.pi, rob.couleur)
            pwm = rob.regulate(t)

            X,V,theta = simule(rob,pwm,h)
            
            rob.setNewState(X,V,theta)

        time.sleep(0.1)
    vibes.endDrawing()
    
    