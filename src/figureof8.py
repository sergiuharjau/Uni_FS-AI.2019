from cmds import issueCommands
from gps import GPS
import math
import time
from geopy import distance

if __name__ == "__main__":


	gps = GPS(logging=False)

	while gps.getCoords() == (0,0):
		print("Awaiting gps lock.")

	startingPos = gps.getCoords() #so we know to stop 15m afterwards

	issueCommands(0,0) #connect to car

	distanceAway = 0
	while distanceAway < 15: #go forward 15m
		distanceAway = (distance.distance(gps.getCoords(), startingPos)).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(0,85)

	issueCommands(0,0) 
	centerGPS = gps.getCoords() 
	time.sleep(3)

	startLeft = issueCommands.car.get_left_pulse() #should be 0
	startRight = issueCommands.car.get_right_pulse() #should be 0 
	startPulse = int((startLeft + startRight)/2)

	flip = 1 #goes right first

	try: 
		while True:
			##get odometry on wheels every frame
			leftPulse=issueCommands.car.get_left_pulse()
			rightPulse=issueCommands.car.get_right_pulse()
			averagePulse = int((leftPulse+rightPulse)/2)

			

			issueCommands(flip*-24, 85) #go slightly right 
			
			

			if (averagePulse - startPulse) > 50: #circumference of circle / wheel circumeference * Pulse/rotation (probably 20)
				if (distance.distance(gps.getCoords(), centerGPS)).m < 2:
					print("Reached center")
					count += 1
					if count == 2:
						count = 0
						flip = -1 #so we go the other way
						print("Flipped")
					else:
						print("First lap. Doing another.")
					startPulse = int((rightPulse + leftPulse) / 2) #reset starting pulse to center again
				else:
					print("Pulses say we should be centered, but GPS doesn't agree.")
					print("GPS: ", gps.getCoords())

				
	except KeyboardInterrupt:
		issueCommands(0,0, True)
#		gps.stop()

		##calculate if we've done a full circle yet 
