import math
import time
import cv2
import logging
import sys

from geopy import distance
import threading
from imProc import imProcessing
from capturing import ImageCap
from cmds import calculateReading, issueCommands
from globals import pixelStrip, startFrom, logInitial
from gps import GPS

if __name__ == "__main__":


	gps = GPS(logging=False)

	while gps.getCoords() == (0,0):
		print("Awaiting gps lock.")
		#time.sleep(0.2)
		gps.getGPS()

	startingPos = gps.getCoords() #so we know to stop 15m afterwards

	issueCommands(0,0) #connect to car

	distanceAway = 0
	while distanceAway < 5: #go forward 15m
		distanceAway = distance.distance(gps.getCoords(), startingPos).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(0,85)
		gps.getGPS()

	issueCommands(0,0)
	centerGPS = gps.getCoords() 
	time.sleep(3)

	startLeft = issueCommands.car.get_left_pulse() #should be 0
	startRight = issueCommands.car.get_right_pulse() #should be 0 
	startPulse = int((startLeft + startRight)/2)

	flip = 1 #goes right first

			
	calculateReading.pastCom = 0  # in case we don't see cones straight away
	logging.basicConfig(filename='loggingMain.log', format='%(asctime)s %(message)s', level=logging.INFO)
	logging.warning("\nMission start!\n")

	visual= False; green= False; record= False; replay= False; loop= True; rc=False; cFlip=0 ;
	eight = True; swapCircles=0 #goes right first
	followGreen=0 

	ic = ImageCap(False, replay)  # ImCapt() #initializes zed object

	for argument in sys.argv[1:]:
		exec(argument)

	startTime = time.time()
	count=0
	try: 
		while True:
			#gps.getGPS()
			##get odometry on wheels every frame
			leftPulse=issueCommands.car.get_left_pulse()
			rightPulse=issueCommands.car.get_right_pulse()
			averagePulse = int((leftPulse+rightPulse)/2)


			image, depth = ic.latest(record)
			logging.info("Getting latest image and depth.")
			t = threading.Thread(target=ImageCap.capture, args=(ic, ))

			original_image = image
			depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
			image_ocv = original_image[startFrom:startFrom + pixelStrip]

			steering, velocity = imProcessing(t, image_ocv, depth_data_ocv, visual, original_image, green, cFlip, eight, swapCircles, followGreen)

			print("Steering: ", steering)
			print("Velocity: ", velocity)
			logging.info("Steering: %d, Velocity: %d", steering, velocity)

			steering = min(19, max(-19, steering))

			issueCommands(steering, velocity)
			print("Pulses done so far: ", averagePulse-startPulse)
			print("Distance traveled: ",distance.distance(gps.getCoords(), centerGPS).m )
			if (averagePulse - startPulse) > 650: #circumference of circle / wheel circumeference * Pulse/rotation (probably 20)
				if (distance.distance(gps.getCoords(), centerGPS).m) < 1:
					print("Reached center")
					count += 1
					if count == 2:
						if swapCircles == 1: # 4th full circle we do
							followGreen = 1
							startingPos = gps.getCoords()
						count = 0
						swapCircles = 0 #so we go the other way
						print("Flipped")
					else:
						print("First lap. Doing another.")
					startPulse = int((rightPulse + leftPulse) / 2) #reset starting pulse to center again
				else:
					print("Pulses say we should be centered, but GPS doesn't agree.")
					print("GPS: ", gps.getCoords())

			if followGreen:
				gps.getGPS()
				distanceAway = distance.distance(gps.getCoords(), startingPos).m
				print("Distance away from destination: ", distanceAway)
				if distanceAway > 15: #after 15m of following green
					raise KeyboardInterrupt

				
	except KeyboardInterrupt:
		issueCommands(0,0)
		time.sleep(4)
		issueCommands(0,0, True)

		print("Seconds it took: ", time.time() - startTime)
		print("Total frames: ", len(listReadings))
		print("Actual framerate: ", len(listReadings) / (time.time() - startTime))
		print("\nFINISH")

		logging.info("\nTotal seconds: %d", time.time()-startTime)
		logging.info("Framerate: %d", len(listReadings) / (time.time() - startTime))
		logging.warning("Mission end.\n\n")
		quit()
		gps.stop()

		##calculate if we've done a full circle yet 
