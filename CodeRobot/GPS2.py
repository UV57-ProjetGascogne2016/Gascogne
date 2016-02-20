__author__      = "SPID 2016"
import time as t
import serial,sys,signal,atexit
import string, threading
import numpy as np
import LatLongUTMconversion as utm
import _thread as thread
from pynmea import nmea



class ThreadGPS1(threading.Thread):
    '''dérivation d'un objet thread pour gérer un GPS'''
    def __init__(self,i,ser,M,Lock,LockEnd):
        threading.Thread.__init__(self)
        self.i = i
        self.M = M
        self.alive = True
        self.ser = ser
        self.Lock = Lock
        self.LockEnd = LockEnd
        
    def run(self):
        initT = t.time()
        with self.LockEnd:
           cont = self.alive
        k=0
        while cont:
            k=k+1
            (lattmp,longtmp,speedtmp,etmp,ntmp,timetmp) = returnLongLat(self.ser[self.i])
            Mtmp = np.array([lattmp,longtmp,speedtmp,etmp,ntmp,timetmp])
            if(self.i==0):
                print("robot ",self.i,":",timetmp)
                print(t.time()-initT)
            with self.Lock:
                self.M[self.i,:] = Mtmp
            t.sleep(1)
            with self.LockEnd:
              cont = self.alive
        # Le thread se termine ici 
        
    def stopT(self):
        print(self.i,"stop")
        with LockEnd:
          self.alive = False
        
class ThreadGPSAll():
    '''Objet lançant tous les threads'''
    def __init__(self,nbGPS,Lock,LockEnd):
        threading.Thread.__init__(self)
        self.nbGPS = nbGPS
        self.M = np.zeros((4,6))
        self.listGPS = []
        self.alive = True
        self.Lock = Lock
        self.LockEnd = LockEnd
        
    def run(self):        
    
        # initialisation des ports
        ser = initGPS()
        # initialisation
        for i in range(self.nbGPS):
            self.listGPS.append(ThreadGPS1(i,ser,self.M,self.Lock,self.LockEnd))
            self.listGPS[i].start()
        with self.LockEnd:
           cont = self.alive
        while cont:
            with self.LockEnd:
              cont = self.alive
        
    def stopT(self):
        with self.LockEnd:
          self.alive = False
        for thin in self.listGPS:
            thin.stopT()

def getGPSDecimalCoordinate(gprmc,data,verbose=False):
    #Return the coordinates in decimal degrees and the speed 
    gprmc.parse(data)
    
    latitude = gprmc.lat
    lats = float(latitude[0:2])
    lats_min = float(latitude[2:])/60
    latitude_deg = lats+lats_min

    longitude = gprmc.lon
    lon = float(longitude[0:3])
    lon_min = float(longitude[3:])/60
    longitude_deg = lon+lon_min
    
    if gprmc.lon_dir  == "W":
        longitude_deg = -longitude_deg
    if gprmc.lat_dir  == "S":
        latitude_deg = -latitude_deg
        
    speed = gprmc.spd_over_grnd
    time = gprmc.timestamp
    
    if verbose:
        print("Latitude values : " + str(latitude_deg))
        print("Longitude values : " + str(longitude_deg))
        print("Speed value : " + str(speed))
        print("time value : " + str(time))
            
    return latitude_deg,longitude_deg,speed,time

def getGPSUTMCoordinate(lat,long,verbose=False):
    (z,e,n) = utm.LLtoUTM(23, lat, long)
    
    if verbose == True:
        print("Easting : "+str(e))
        print("Northing : "+str(n))
        
    return e,n
    
def openPort(port,baudrate=4800):
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.timeout = 1
        ser.open()
    except serial.serialutil.SerialException as e:
        # # print("error opening port "+ser.port+ str(e))
        1+1
    return ser


        
            
def returnLongLat(ser):
    gprmc = nmea.GPRMC()
    tmp = ""
    timeout = t.time() + 1
    ser.flush()
    while((tmp != '$GPRMC')&(t.time() < timeout)):
        data = str(ser.readline())
        tmp = data[2:8]
        
    gprmc.parse(data)
    
    if data[2:8] !='$GPRMC':
         # print("no tram GRMC detected")
         e=0;n=0;lat=0;long=0;speed=0;time=0
        
    if data[2:8] == '$GPRMC':
        e=0;n=0;lat=0;long=0;speed=0;time=0
        try:
            float(gprmc.lat)
            float(gprmc.lon)
            float(gprmc.spd_over_grnd)
            float(gprmc.timestamp)
            [lat,long,speed,time] = getGPSDecimalCoordinate(gprmc,data,False)
            (e,n) = getGPSUTMCoordinate(lat,long)
                
        except:
            # print("reception problem")
            flag=False
            return lat,long,speed,e,n,time
    
    return lat,long,speed,e,n,time

def returnAllLatLong(ser,nbrGPS):
    lat=[];long=[];speed=[];e=[];n=[];time=[];
    M = np.zeros((nbrGPS,6))
    tt = t.time()
    for i in range(0,nbrGPS):
        (lattmp,longtmp,speedtmp,etmp,ntmp,timetmp) = returnLongLat(ser[i])
        Mtmp = np.array([lattmp,longtmp,speedtmp,etmp,ntmp,timetmp])
        M[i,:] = Mtmp
    print(t.time()-tt)
    return(M)
    
def initGPS():
    ser = []
    ser.append(openPort("COM3"))
    ser.append(openPort("COM12"))
    ser.append(openPort("COM5"))
    ser.append(openPort("COM6"))
    return ser
    
def endFunction(th):
    print("Threads closed!")
    
    

def signal_handler(signal,frame):
    print("Interuption detected !")
    MainThread.stopT()
    sys.exit(0)
    
if __name__ == '__main__':
    Lock = threading.RLock()
    LockEnd = threading.RLock()
    nbrGPS = 4
    # initialise le thread principal
    MainThread = ThreadGPSAll(nbrGPS,Lock,LockEnd)
    
    # Programme la gestion de l'interruption
    signal.signal(signal.SIGINT, signal_handler)
    atexit.register(endFunction,MainThread)
    # M = returnAllLatLong(ser,nbrGPS)
    
    MainThread.run()



