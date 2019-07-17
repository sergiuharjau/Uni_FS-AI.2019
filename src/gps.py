import serial
import pynmea2
import time
import threading
from datetime import datetime

class GPS():

    def __init__(self, logging=False):
        self.coords = (0,0)
#        self.running = True
        self.log = logging
#        self.t = threading.Thread(target=GPS.getGPS, args=(self, ))
#        self.t.start()
    
    def stop(self):
        self.running = False
        self.logGPS(True)
#        self.t.join()

    def getCoords(self):
        return self.coords

    def logGPS(self, write=False):

        listGPS.append(self.coords)
        
        if write:
            f1= open("gpsData.txt", "a+")
            for coords in listGPS:
                print("Writing to filesystem")
                x,y = coords
                f1.write(str(datetime.now()) + "," + str(x) + "," + str(y) + "\n")
            f1.close()

    def getGPS(self):

        ser = serial.Serial("/dev/ttyACM1")

        dataout = pynmea2.NMEAStreamReader()

        while 1:
                newdata = ser.readline()        # Read a line from serial device
                print(newdata[:6])
                if newdata[0:6] == "$GPGGA".encode():        # If line contains GPS coordinate info
                    newmsg = pynmea2.parse(newdata.decode()) # Parse it
                    self.coords = (newmsg.latitude, newmsg.longitude)
                    if self.log:
                        self.logGPS()
                    return self.coords #time.sleep(0.5) #when we find, take a little break
                #else:
                   # time.sleep(0.02) #keep reading until we find it

if __name__ == "__main__":

    getGPS()
