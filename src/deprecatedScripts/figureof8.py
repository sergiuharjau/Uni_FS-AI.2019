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
		gps.getGPS(force=1)

	startingPos = gps.getCoords() #so we know to stop 15m afterwards

	issueCommands(0,0) #connect to car

	distanceAway = 0
	while distanceAway < 12: #go forward 15m
		distanceAway = distance.distance(gps.getGPS(force=1), startingPos).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(1,100) #go straight

	issueCommands(0,0)


	startLeft = issueCommands.car.get_left_pulse() #should be 0
	startRight = issueCommands.car.get_right_pulse() #should be 0 
	startPulse = int((startLeft + startRight)/2)

	flip = 1 #goes right first
			
	calculateReading.pastCom = 0  # in case we don't see cones straight away
	logging.basicConfig(filename='loggingMain.log', format='%(asctime)s %(message)s', level=logging.INFO)
	logging.warning("\nMission start!\n")

	visual= False; green= False; record= False; replay= False; loop= True; rc=False; cFlip=0 ;
	swapCircles=0 #goes right first

	ic = ImageCap(False, replay)  # ImCapt() #initializes zed object

	for argument in sys.argv[1:]:
		exec(argument)

	time.sleep(5)
	startTime = time.time()
	timeMarker = False
	count=0
	timeMarker = time.time()
	centerGPS = gps.getGPS(force=1)
	closeEyes = 7
	try: 
		while True:
			#gps.getGPS()
			##get odometry on wheels every frame
			leftPulse=issueCommands.car.get_left_pulse()
			rightPulse=issueCommands.car.get_right_pulse()
			averagePulse = int((leftPulse+rightPulse)/2)

			#if not passedStart:
			#	if distanceAway = distance.distance(gps.getGPS(force=1), startingPos).m > 7:
			#		issueCommands(0,0)
			#		centerGPS = gps.getCoords()
			#		passedStart = True
			#		timeMarker = time.time()

			image, depth = ic.latest(record)
			logging.info("Getting latest image and depth.")
			t = threading.Thread(target=ImageCap.capture, args=(ic, ))

			original_image = image
			depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
			image_ocv = original_image[startFrom:startFrom + pixelStrip]

			steering, velocity = imProcessing(t, image_ocv, depth_data_ocv, visual, original_image, green, cFlip, swapCircles=swapCircles)

			#print("Steering: ", steering)
			#print("Velocity: ", velocity)
			logging.info("Steering: %d, Velocity: %d", steering, velocity)


			if not gps.running:
				t = threading.Thread(target=GPS.getGPS, args=(gps, 0.7))
				t.start()

			distance1 = gps.getCoords()

			print("Distance to center: ", distance1)
				
			if distance1 < closeEyes:
				steering = 10 * flip

			velocity=100
			steering = min(19, max(-19, steering))

			issueCommands(steering, velocity)

			if timeMarker - time.time() > 5: #looks 20s after it leaves the center
				if distance1 < 1:
					timeMarker = time.time()
					print("Reached center")
					count += 1
					if count == 2:
						if flip == -1: # 4th full circle we do
							closeEyes = -1
						count = 0
						flip=-1
						print("Flipped")
					else:
						print("\n\n\nFirst lap. Doing another.")

			if closeEyes == -1:
				if distance1 > 3:
					raise KeyboardInterrupt

	except KeyboardInterrupt:
		issueCommands(0,0)
		time.sleep(3)

		while (distance.distance(gps.getGPS(force=1), centerGPS).m) < 15:
			issueCommands(1, 100)
			time.sleep(1)
		issueCommands(0,0)
		time.sleep(5)
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
