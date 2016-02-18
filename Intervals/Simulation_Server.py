
# Définition d'un serveur réseau gérant un système de CHAT simplifié.
# Utilise les threads pour gérer les connexions clientes en parallèle.

HOST = '127.0.0.1'
PORT = 7000
import socket,threading,pickle
import sys,signal,atexit
import time
from simulationControl import *

global M # None
M = []
Lock = threading.RLock()
LockEnd = threading.RLock()

nbsRobots = 10 # number of boats
Tf = 5000 # end of simulation in s
dt = 1 # simulation step in s
tps_ellipse=500 # system speed (ellipse goal reached after tps_ellipse) in m/s
period_ellipse = 200 # elliptical period of a robot in s




class ThreadSimu(threading.Thread):
    '''dérivation d'un objet thread pour gérer la connexion avec un client'''
    def __init__(self):
        global M
        threading.Thread.__init__(self)
        
        with Lock:
            for i in range(nbsRobots):
               M.append([0,[0,0]])
        # Initialisation Simulation
        self.simu = SimulationControl(nbsRobots,Tf,dt,tps_ellipse,period_ellipse)
        self.alive = True
        
    def run(self):
        global M
        theta_i = 0 # initial angle of the ellipse
        l_i = 0.1 # initial major axis of the ellipse
        centre_i = np.array([[0],[0]]) # initial center of the ellipse
        k = 0 # start with position number 0
        tk = 0
        l_f,theta_f,centre_f = self.simu.param_ellipse(self.simu.objectifs,k) # ellipse parameter to reach
        x0 = np.array([[0],[0],[0],[1]])
        X = np.dot(x0,np.ones((1,self.simu.N))) # position of the robots

 
        for t in np.arange(0,self.simu.Tf,self.simu.dt):
            t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f, positions, X ,cons = self.simu.simuOneStep(t,k,tk,theta_i,theta_f,l_i,l_f,centre_i,centre_f,X)
            with Lock:
               M = positions
            time.sleep(self.simu.dt)
            with LockEnd:
              cont = self.alive
            if not cont:
               break
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
            data_string = pickle.dumps(arr)
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
