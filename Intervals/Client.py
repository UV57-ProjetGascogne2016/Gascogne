# Définition d'un client réseau rudimentaire
# Ce client dialogue avec un serveur ad hoc

import socket, sys, pickle


class Client:
     def __init__(self, HOST = '192.168.8.201',PORT = 50000):
         # 1) création du socket :
         self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

         # 2) envoi d'une requête de connexion au serveur :
         try:
           self.mySocket.connect((HOST, PORT))
         except socket.error:
           print("La connexion a échoué.")
           sys.exit()    
         print("Connexion établie avec le serveur.")
         # 3) Dialogue avec le serveur :
         self.msgServeur = self.mySocket.recv(1024)
         print (self.msgServeur)

     def getData(self):
         

         if self.msgServeur.upper() == ("FIN").encode() or self.msgServeur ==("").encode():
            return []
         self.mySocket.send(("E").encode())
         self.msgServeur = self.mySocket.recv(4096)
         if not self.msgServeur: 
            return []
         Array =pickle.loads( self.msgServeur)
         print("reception de l'array : ",Array)
         return Array

     def close(self):
        # 4) Fermeture de la connexion :
        print("Connexion interrompue.")
        self.mySocket.close()
