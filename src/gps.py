import serial
import pynmea2
import time
import threading
from datetime import datetime

class GPS():

    def __init__(self):
        self.coords = (0,0)
        self.running = True
        self.t = threading.Thread(target=GPS.getGPS, args=(self, ))
        self.t.start()
    
    def stop(self):
        self.running = False
        self.t.join()

    def getCoords(self):
        return self.coords

    def logGPS(self, x=None, y=None):
        if x == None:
            x,y=self.coords
        f1= open("gpsData.txt", "a+")
        f1.write(str(datetime.now()) + "," + str(x) + "," + str(y) + "\n")
        f1.close()

    def getGPS(self):

        ser = serial.Serial("/dev/ttyACM1")

        dataout = pynmea2.NMEAStreamReader()

        while self.running:
                newdata = ser.readline()        # Read a line from serial device
                if newdata[0:6] == '$GPGGA':        # If line contains GPS coordinate info
                    newmsg = pynmea2.parse(newdata) # Parse it
                    self.coords = (newmsg.latitude, newmsg.longitude)
                    time.sleep(0.1)
                else:
                    time.sleep(0.1)

if __name__ == "__main__":

    getGPS()