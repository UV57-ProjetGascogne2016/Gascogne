import serial
import pygame
from pygame.locals import *
import testserial
import time
import _thread as thread
import atexit
import signal
import sys
import numpy as np
import threading


class command(threading.Thread):
    def __init__(self):
        

        threading.Thread.__init__(self)
        # Seule la variable ListRobots sera accessible!
        robot1=[3,7] #3 marche avant et 7 rotation 
        robot2=[1,0] #1 marche avant et 0 rotation
        robot3=[4,6] #4 marche avant et 6 rotation
        robot4=[2,5] #2 marche avant et 5 rotation
        self.ListRobots=[robot1,robot2,robot3,robot4]

        
        # Neutre defini empiriquement pour chaque robot
        neutral1=[1500,1500] # 3 7
        neutral2=[1500,1500] # 1 0 
        neutral3=[1500,1500] # 4 6
        neutral4=[1500,1500] # 2 5
        self.Neutral=[neutral1,neutral2,neutral3,neutral4]
        
        #self.Pwmforw=[1650,1650,1650,1650]     
        self.Pwmforw=[1500,1500,1500,1500]    #Valeur de départ
        self.Pwmturn=[1500,1500,1500,1500]
        self.Deltafw=[0,0,0,0]
        self.Deltaturn=[0,0,0,0]

        self.nbRobot = 4
        self.Vmin=1000
        self.Vmax=2000
        self.consmin = 1000
        self.consmax = 2000
        self.port = 'COM8'
        
        #self.consigne = self.Neutral
        #self.consigne = [[1500,1501], [1502,1503], [1504,1505], [1506 ,1507]]
        self.consigne = [[1500,1800], [1500,1800], [1500,1800], [1500 ,1800 ] ]
        
        #### Fin de l'initialisation des variables
        
        try:
            self.ser = serial.Serial(self.port)
            self.ser.open()
            self.ser.write(chr(0xAA))
            self.ser.flush()
        except serial.serialutil.SerialException as e:
            print(": Error opening or initialising port "+self.port+" : "+str(e))
            
        ## init of the thread value
        pygame.init()
        window = pygame.display.set_mode((640,480))
        pygame.display.flip()
        atexit.register(endFunction,self)
        signal.signal(signal.SIGINT, signal_handler)  
        ###### Initialize the joysticks
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        pygame.joystick.init()
        self.axes = self.joystick.get_numaxes() 
        self.auto = 1
        
    def run(self):
        
        
        ## run all the programm
        if self.auto == 1:
            while 1:
                invturn = (False,False,False,False) # ok
                invforw = (True,False,True,False) # ok
                consigne = self.consigne
                for i in range(self.nbRobot):
                    geteventauto(self,i,invforw,invturn)
                self.send_all_command(self.nbRobot,self.ListRobots, self.Pwmforw, self.Pwmturn)
                
        elif self.auto == 0:
            t=0
            invturn = (True,True,False,True) # 1 2 3 et 4 ok !
            invforw = (True,True,False,False) # 1 2 3 et 4 ok !
            while 1:
                self.send_all_command(self.nbRobot,self.ListRobots, self.Pwmforw ,self.Pwmturn)
                for i in range(c.nbRobot):
                    thread.start_new_thread(geteventjoystick,(self,i,invforw,invturn,joystick))
                    #geteventjoystick(c,i,invforw,invturn,joystick)
        else :
            print('mode de fonctionnement non reconnu')
    
    
    def get_commandPololu(self,channel, target):
            target = target * 4
            serialBytes = chr(0x84)+chr(channel)+chr(target & 0x7F)+chr((target >> 7) & 0x7F)
            return serialBytes

    def send_command(self,robot,pwmf,pwmt):
        serialBytes = self.get_commandPololu(robot[0], pwmf)
        self.ser.write(bytes(serialBytes,'utf-8'))
        serialBytes = self.get_commandPololu(robot[1], pwmt)
        self.ser.write(bytes(serialBytes,'utf-8'))
        
    def send_all_command(self,nbRobot, tabRobot, tabPwmf, tabPwmt):
        num_targets = 8
        start_channel = 0
        result = []
        indices = [3,2,6,0,4,7,5,1]
        #indices = [0,1,2,3,4,5,6,7]
        cons=[0,0,0,0,0,0,0,0]
        values = [0,0,0,0,0,0,0,0]
        j=0
        
        for i in range(nbRobot):# pour remplir la consigne dans le désordre
            cons[2*i]=tabPwmf[i]
            cons[2*i+1]=tabPwmt[i]
            
        
        for i in indices: # pour mettre les consignes dans l'ordre (5,1,3,0,2,7,6,4)
            values[j] = cons[i]
            j = j+1
            
        
        for k in range(8):
            lowbits = int(values[k])*4 & 0x7F
            highbits = (int(values[k])*4 >> 7) & 0x7F
            
            result.append(lowbits)
            result.append(highbits)
        
        self.write(0xAA,12,0x1F,num_targets,start_channel,result)
          
    def write(self,*data):
        
        for d in data:
            if type(d) is list:
                # Handling for writing to multiple servos at same time
                for li in d:
                    self.ser.write(bytes([li]))
            else:
                self.ser.write(bytes([d]))
 
        self.ser.flush()
        
    def dectohex(self,nb):
        reste = nb%256
        if nb>255:
            low = hex(reste)
            hight = hex(int((nb-reste)/256))
        else:
            low = hex(reste)
            hight = hex(0)
        return(low,hight)
        
    def set_neutral(self,robot,neutral):
        serialBytes = self.get_commandPololu(robot[0], neutral[0])
        self.ser.write(bytes(serialBytes,'utf-8'))
        serialBytes = self.get_commandPololu(robot[1], neutral[1])
        self.ser.write(bytes(serialBytes,'utf-8'))
    
    def set_command_joystick(self,minserv,maxserv,minjoy,maxjoy,currentJoy,inv=False):
        if inv:
            a = (minserv-maxserv)/(minjoy-maxjoy)
            b = minserv - ((maxserv-minserv)/(maxjoy-minjoy))*minjoy
            output = round(a*currentJoy + b)
            return output
        else:
            a = (maxserv-minserv)/(minjoy-maxjoy)
            b = maxserv - ((minserv-maxserv)/(maxjoy-minjoy))*minjoy
            output = round(a*currentJoy + b)
            return output
            
    def setConsigne(self,forw1,turn1,forw2,turn2,forw3,turn3,forw4,turn4):
        self.consigne[0][0] = forw1
        self.consigne[1][0] = forw2
        self.consigne[2][0] = forw3
        self.consigne[3][0] = forw4
        self.consigne[0][1] = turn1
        self.consigne[1][1] = turn2
        self.consigne[2][1] = turn3
        self.consigne[3][1] = turn4
        
    def close(self):
        self.ser.close()

def geteventjoystick(c,numRobot,invforw,invturn,joystick):
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            if (event.dict['axis'] == 1):
                axis1 = joystick.get_axis(1)
                c.Pwmturn[numRobot]=c.set_command_joystick(1000+c.Deltaturn[numRobot],2000+c.Deltaturn[numRobot],-0.586,0.611,axis1,invturn[numRobot])
                
            if (event.dict['axis'] == 3):
                axis3 = joystick.get_axis(3)
                c.Pwmforw[numRobot] = c.set_command_joystick(1000+c.Deltafw[numRobot],2000+c.Deltafw[numRobot],-0.766,0.778,axis3,invforw[numRobot])
        
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                Pwmforw[numRobot] = neutral[numRobot][0]
                Pwmturn[numRobot] = neutral[numRobot][1]
                c.send_command(Robot[numRobot],Pwmforw[numRobot], Pwmturn[numRobot])
                
                time.sleep(1)
                cond = 0
                print('end robot ' + c.nbRobot)
                
def geteventauto(c,i,invforw,invturn):
    c.Pwmforw[i] = c.set_command_joystick(c.Vmin+c.Deltafw[i],c.Vmax+c.Deltafw[i],c.consmin,c.consmax,c.consigne[i][0],invforw[i])
    c.Pwmturn[i] = c.set_command_joystick(c.Vmin+c.Deltaturn[i],c.Vmax+c.Deltaturn[i],c.consmin,c.consmax,c.consigne[i][1],invturn[i])

# Fonction pour arreter les robots en cas d'interruption ou de pb
def endFunction(c):
    for i in range(c.nbRobot):
        print(i)
        c.set_neutral(c.ListRobots[i],c.Neutral[i])
        time.sleep(0.1)
    time.sleep(1)

# détection de l'interruption
def signal_handler(signum, frame):
    print("Interuption detected !")
    sys.exit(0)

def initVarCommunication():
    ##init
    pygame.init()
    window = pygame.display.set_mode((640,480))
    pygame.display.flip()
    c = testserial.command()
    atexit.register(endFunction,c)
    signal.signal(signal.SIGINT, signal_handler)  
    ###### Initialize the joysticks
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    pygame.joystick.init()
    axes = joystick.get_numaxes() 
    return c,joystick , axes

         

def run2(auto,c,joystick):
    ## run all the programm
    if auto == 1:
        while 1:
            invturn = (False,False,False,False) # ok
            invforw = (True,False,True,False) # ok
            consigne = c.consigne
            for i in range(c.nbRobot):
                geteventauto(c,i,invforw,invturn)
            c.send_all_command(c.nbRobot,c.ListRobots, c.Pwmforw, c.Pwmturn)
            
    elif auto == 0:
        t=0
        invturn = (True,True,False,True) # 1 2 3 et 4 ok !
        invforw = (True,True,False,False) # 1 2 3 et 4 ok !
        while 1:
            c.send_all_command(c.nbRobot,c.ListRobots, c.Pwmforw ,c.Pwmturn)
            for i in range(c.nbRobot):
                thread.start_new_thread(geteventjoystick,(c,i,invforw,invturn,joystick))
                #geteventjoystick(c,i,invforw,invturn,joystick)
    else :
        print('mode de fonctionnement non reconnu')
        
        
        
        
        
if __name__=='__main__':
    
    # c,joystick,axes = initVarCommunication()
    # auto = 1
    # run2(auto,c,joystick)
    
    Main = command()
    
    
    Main.run()





        


