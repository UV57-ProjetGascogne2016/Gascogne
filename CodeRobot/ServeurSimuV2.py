
# Définition d'un serveur réseau gérant un système de CHAT simplifié.
# Utilise les threads pour gérer les connexions clientes en parallèle.

HOST = '169.254.178.135'   #'192.168.8.201'
PORT = 7000

import socket,threading,pickle
import sys,signal,atexit
import time
import serial

from GPS2 import *
from regulateur import *
from testserial import *
from SimulationControl import *

global M # None
M = np.zeros((4,6))
Lock = threading.RLock()
LockEnd = threading.RLock()

class ThreadSimu(threading.Thread):
    '''dérivation d'un objet thread pour gérer la connexion avec un client'''
    def __init__(self):
        global M
        global Lock
        global LockEnd
        
        # init the server
        threading.Thread.__init__(self)
        self.nbsRobots = 4
        self.listRobots = []
        # self.Command = command()
        self.listRobotsReels = [] #self.Command.ListRobots
        
        # Initialisation Simulation
        self.simu = SimulationControl(self.nbsRobots,550,20)
        self.alive = True
        
        # objet initialisant les threads des GPS
        self.MainThread = ThreadGPSAll(self.nbsRobots,Lock,LockEnd)
        
    def run(self):
        global M
        global Lock
        global LockEnd
        theta_i = 0 # initial angle of the ellipse
        l_i = 0.1 # initial major axis of the ellipse
        centre_i = np.array([[0],[0]]) # initial center of the ellipse
        k = 0 # start with position number 0
        tk = 0
        l_f,theta_f,centre_f = self.simu.param_ellipse(self.simu.objectifs,k) # ellipse parameter to reach
        x0 = np.array([[0],[0],[0],[1]])
        X = np.dot(x0,np.ones((1,self.simu.N))) # position of the robots
        ListPWM = []
        self.MainThread.run()
        # Init affichage
        vibes.beginDrawing()
        vibes.newFigure('Simulation Gascogne avec Terrain de Foot')
        vibes.setFigureProperties({'x': 200, 'y': 100, 'width': 800, 'height': 800})
        points = [[0,0],[40.863285863772035, 4.2179290521889925],[37.32119551504729, 39.42228888720274],[-3.5161916106007993, 36.53818473871797]]
        objectifs = [[0,0],[40.863285863772035, 4.2179290521889925],[37.32119551504729, 39.42228888720274],[-3.5161916106007993, 36.53818473871797]]
        vibes.drawLine(points) # drawing of the coast
        vibes.drawLine(objectifs,'green')  # drawing of ellipse target bounds 

        
        for i in range(0,self.nbsRobots):
            self.listRobots.append(Robot(-4.0,45.0,0.0,0.0,i,'b',self.nbsRobots))
            
        for t in np.arange(0,self.simu.Tf,self.simu.dt):
            
            ## Récupère la postion des robots réels
            with Lock:
               M = self.MainThread.M
               print(self.MainThread.M)
               Arr = M
               print(M)
            #time.sleep(self.simu.dt)
            
            
            ## Affichage des robot
            ref = [391039.7802301956,  5363846.097328593]
            vibes.clearFigure()
            vibes.drawLine(points, 'black')
            vibes.drawLine(objectifs,'green')
            for i in range(self.nbsRobots):
                vibes.drawCircle(Arr[i,3]-ref[0],Arr[i,4]-ref[1],0.5,'blue')
            
            ## mettre a jour coordonnées du robot
            for i in range(self.nbsRobots):
                # cas où x,y = lat,lon
                #self.listRobots[i].setNewState(np.array(Arr[i,0],Arr[i,1]),Arr[i,2])
                # cas où x,y = east,north
                self.listRobots[i].setNewState(np.array([Arr[i,3],Arr[i,4]]),Arr[i,2])
                theta = self.listRobots[i].thetaMesure
                X[0,i]=Arr[i,3]
                X[1,i]=Arr[i,4]
                X[2,i]=Arr[i,2]*np.cos(theta)+1
                X[3,i]=Arr[i,2]*np.sin(theta)+1
                # Probleme probable ici car orientation de theta a revoir
            
            # calcule la consigne
            t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, positions, X ,cons,dcons = self.simu.simuOneStep(t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f,X)
            # X correspond à la position théorique des robots, cons[-self.nbsRobots] correspond à la dernière consigne calculée pour les robots
            
            
            for i in range(self.nbsRobots):
                ## Reception consigne PWM
                PWMrobot = self.listRobots[i].regulate(np.array([cons[i][0,0],cons[i][1,0]]),np.array([dcons[i][0,0],dcons[i][1,0]]))
                ListPWM.append(PWMrobot)
                 
            print(ListPWM)
            
            for i in range(self.nbsRobots):
                ## completer avec le code de commande robot
                # commande à utiliser pour normaliser la commande
                # appliquqe la commande trouvée
                # setPWM(PWM) :: append à la liste Pwnforw et Pwmturn les valeurs de PWM[0] et PWM[1]
                1+1
                
            ListPWM.clear()
            
            with LockEnd:
              cont = self.alive
            if not cont:
               break
               
        vibes.endDrawing()
        with Lock:
            M = []
        
    def stopT(self):
        with LockEnd:
          self.alive = False
           
            
class ThreadClient(threading.Thread):
    '''dérivation d'un objet thread pour gérer la connexion avec un client'''
    def __init__(self, conn):
        threading.Thread.__init__(self)
        self.connexion = conn
        self.alive = True
        
    def run(self):
        nom = self.getName()
        global M
        global Lock
        global LockEnd
        with LockEnd:
           cont = self.alive
        while cont:
            msgClient = self.connexion.recv(1024)
            if msgClient.upper() == ("FIN").encode() or msgClient ==("").encode():
                self.connexion.send(("FIN").encode())
                break
            msgClient.upper() == ("E").encode()
            with Lock:
                arr = M.copy()
                
            ListaData = self.envoieList(arr)
            print(ListaData)
            data_string = pickle.dumps(ListaData)
            self.connexion.send(data_string)
            with LockEnd:
               cont = self.alive
                   
        # Fermeture de la connexion :
        self.connexion.close()      # couper la connexion côté serveur
        del conn_client[nom]        # supprimer son entrée dans le dictionnaire
        print("Client ",nom ," déconnecté.")
        # Le thread se termine ici 
    def stopT(self):
        with LockEnd:
          self.alive = False
          
    def envoieList(self,arr):
        L=[]
        for i in range(4):
            L.append([arr[i,5],arr[i,3],arr[i,4]])
        return L

def endFunction():
    print("closing connexions !")
    mySocket.close()
    for cle in conn_client:
        conn_client[cle].close()
    
    

def signal_handler(signal, frame):
    print("Interuption detected !")
    for th in thList:
        th.stopT()
    sys.exit(0)

if __name__ == '__main__':

   atexit.register(endFunction)
   signal.signal(signal.SIGINT, signal_handler)
   # Initialisation du serveur - Mise en place du socket :
   mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   try:
      mySocket.bind((HOST, PORT))
   except socket.error:
      print("La liaison du socket à l'adresse choisie a échoué.")
      sys.exit()
   print("Serveur prêt, en attente de requêtes ...")
   mySocket.listen(5)

   # Attente et prise en charge des connexions demandées par les clients :
   conn_client = {}                # dictionnaire des connexions clients
   # creation d'un thread pour la simulation
   thSimu = ThreadSimu()
   thSimu.start()
   thList = [thSimu]

   while 1:
    connexion, adresse = mySocket.accept()
    # Créer un nouvel objet thread pour gérer la connexion :
    th = ThreadClient(connexion)
    th.start()
    thList.append(th)
    # Mémoriser la connexion dans le dictionnaire : 
    it = th.getName()        # identifiant du thread
    conn_client[it] = connexion
    print ("Client ",it," connecté, adresse IP : " , adresse[0] ,", port : " , adresse[1],".")
    # Dialogue avec le client :
    connexion.send(("Vous êtes connecté. Envoyez vos messages.").encode())
