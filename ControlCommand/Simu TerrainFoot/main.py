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
    T = 150
    h = 0.05
    nombreRobots = 1
    listRobots = []
    for i in range(0,nombreRobots):
        listRobots.append(Robot(48.4188, -4.4743,0.0,0.0,i,'b',nombreRobots))

    vibes.beginDrawing()
    vibes.newFigure('TerrainFoot')
    vibes.setFigureProperties({'x':48, 'y':-4, 'width':800, 'height':800})
    vibes.axisLimits(48.41801,48.419,-4.4733,-4.47447,'TerrainFoot')
    vibes.clearFigure('TerrainFoot') 
    #Golfe de Gascogne
    #vibes.drawLine([[-4.4,47.8],[-3.8,47.8],[-2.3,47.2],[-2,46.6],[-1.3,44.4],[-2.0,43.3],[-3.6,43.5],[-2.8,43.4]])
    #Terrain de foot de l'ENSTA Bretagne
    vibes.drawLine([[48.418928, -4.474467],[48.418980, -4.473588],[48.418046, -4.473384],[48.418018, -4.474322],[48.418928, -4.474467]])
    
    for t in frange(0,T,h):
        # DEROULEMENT pour la simulation:
        # u = Commande renvoyée par chaque régulateur pour chaque bateau
        # Une itération de la simulation grâce à u
        # On change les positions des bateaux
        # On recommence
        
        # DEROULEMENT pour le test réel :
        # calcul de la commande u pour chaque robot par rapport à sa position et la position qu'il doit rejoindre
        # après un temps t (à définir, par exemple 1 s), on récupère la position du robot
        # On récupère un tableau qui contient les informations pour chaque robot à chaque ligne (1 ligne pour 1 robot) : dans l'ordre [lat, long, easting, northing, speed, time]
        # on recommence avec la nouvelle position que doit attendre le robotd
        for rob in listRobots:
            
            vibes.drawAUV(float(rob.X[0]), float(rob.X[1]), 0.005, rob.thetaReel*180/math.pi, rob.couleur)
            pwm = rob.regulate(t)

            X,V,theta = simule(rob,pwm,h)
            # le simulateur sera remplacé par les coordonnées gps reçus pour chaque robot
            
            rob.setNewState(X,V,theta)

        time.sleep(0.1)
    vibes.endDrawing()
    
    