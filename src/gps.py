import serial
import pynmea2
import time

def getGPS():

	ser = serial.Serial("/dev/ttyACM1")
	
	dataout = pynmea2.NMEAStreamReader()

	while 1:
			newdata = ser.readline()		# Read a line from serial device
			if newdata[0:6] == '$GPGGA':		# If line contains GPS coordinate info
				newmsg = pynmea2.parse(newdata)	# Parse it
				return newmsg.latitude, newmsg.longitude

if __name__ == "__main__":

	getGPS()