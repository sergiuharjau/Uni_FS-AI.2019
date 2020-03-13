import time 
import sys 
import numpy as np 
import cv2 
import serial 
import logging

from globals import width, newComOffset, missedColourOffset, maxSpeedUp, steeringFactor, carVelocity

#sys.path.insert(0, '../../fspycan/lib/') #fork from DavidCroft's fspycan github repo.
#import fspycan_ext

missedRed = 0
missedYellow = 0
def findLineMarkers(red, yellow, i, visual):

	global missedRed
	global missedYellow

	logging.info("Processing gate %d",i)

	redIndex = np.where(red == 255)

	redMarker = False
	try:
		redMarker = (redIndex[1][-1], 0)
		missedRed = 0
	except:
		missedRed += 1
		logging.info("MissedRed: %d", missedRed)
		if missedRed > 7:
			print("Can't see blue, turning left", (-1*missedRed*missedColourOffset))
			redMarker = (int(-1 * missedColourOffset * missedRed), 0)
			
			# very far left red, middle yellow, turns left
		else:
			print("Missed Red: ", missedRed)

	yellowIndex = np.where(yellow == 255)
	yellowMarker = False
	
	offset = 0 if isinstance(redMarker, bool) else redMarker[0]

	for element in yellowIndex[1]:
		if element > offset+100: #at least 100px to the right of our blue cone
			yellowMarker = (element, 0)
			missedYellow = 0
			break

	if isinstance(yellowMarker, bool):
		missedYellow += 1
		logging.info("MissedYellow: %d", missedYellow)
		if missedYellow > 7:
			print("Can't see yellow, turning right", (missedYellow*missedColourOffset))
			yellowMarker = (int(1280 + missedColourOffset * missedYellow), 0 )
					# middle red, very far right Yellow, turns right
		else:
			print("Missed Yellow: ", missedYellow)

	if visual:
		for x in range(i + 1):
			cv2.imshow("gate " + str(i), red + yellow)
	print("Red Marker: ", redMarker)
	print("Yellow Marker: ", yellowMarker)
	logging.info("Red marker: %s", str(redMarker))
	logging.info("Yellow marker: %s", str(yellowMarker))
	return redMarker, yellowMarker


def calculateReading(gateDict):

	totalValue = 0 
	cameraValue = 0
	print("Gates: ", len(gateDict))

	for key in gateDict:
		target = gateDict[key][0][0] #pastValue - currentValue
		cameraValue = target - int(width / 2) #define currentValue

		logging.info("Gate %d: CameraValue: %d", key, cameraValue)
		totalValue += cameraValue

	if len(gateDict): #avoids division by 0 error
		averageValue = totalValue/len(gateDict)
		logging.info("Average gate value: %d", averageValue)

		steering = calculateReading.pastCom + (averageValue - calculateReading.pastCom) / newComOffset
		logging.info("Rolling average final camera value: %d", steering)

		averageValue += steeringFactor * 1.5 #2 degrees of steering factor

		averageValue = max(0, min(abs(averageValue), 100))

		if abs(averageValue) < steeringFactor * 3:
			velocity = carVelocity + maxSpeedUp * len(gateDict)/4
		else:
			velocity = carVelocity + (maxSpeedUp+30)*(100-averageValue)/100 * len(gateDict)/4
	else:							#so it is linearly throughout 2-6degrees
		logging.info("No valid gates. Using past command.")
		velocity = carVelocity #coast if no gates
		steering = calculateReading.pastCom #keep past direction

	calculateReading.pastCom = steering
	return round(steering/steeringFactor), round(velocity)
