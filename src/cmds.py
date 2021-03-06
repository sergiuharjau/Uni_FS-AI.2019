import time 
import sys 
import numpy as np 
import cv2 
import serial 
import logging

from globals import width, newComOffset, missedColourOffset, maxSpeedUp, steeringFactor, carVelocity

sys.path.insert(0, '../../fspycan/lib/') #fork from DavidCroft's fspycan github repo.
import fspycan_ext

missedRed = 0
missedYellow = 0
def findLineMarkers(red, yellow, i, visual):

	global missedRed
	global missedYellow

	logging.info("Processing gate %d",i)

	redIndex = np.where(red == 255)
	try:
		redMarker = (redIndex[1][-1], redIndex[0][0])
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
			redMarker = False

	yellowIndex = np.where(yellow == 255)
	try:
		yellowMarker = (yellowIndex[1][0], yellowIndex[0][0])
		missedYellow = 0
	except:
		missedYellow += 1
		logging.info("MissedYellow: %d", missedYellow)
		if missedYellow > 7:
			print("Can't see yellow, turning right", (missedYellow*missedColourOffset))
			yellowMarker = (int(1280 + missedColourOffset * missedYellow), 0 )
			# middle red, very far right Yellow, turns right
		else:
			print("Missed Yellow: ", missedYellow)
			yellowMarker = False

	if visual:
		for x in range(i + 1):
			cv2.imshow("gate " + str(i), red + yellow)
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

def issueCommands(steering=0, velocity=0, exit=False, visual=False, replay=False, record=False, rc=False):

	if not replay and not record and not visual and not rc:
		if 'car' not in issueCommands.__dict__:  # only runs once
			issueCommands.car = fspycan_ext.Car("can0")
			issueCommands.car.init()

			print("Initiating CAN setup.")
			logging.info("Setting up can Device.")
			issueCommands.car.setupCAN()  # function runs until we finish setup
			print("Setup finished gracefully")

		logging.info("Setting steering and velocity.")
		issueCommands.car.set_steering_velocity(int(steering*-1), int(velocity))
		logging.info("CAN data set.")
		# we only set the steering here, the loop runs on a different c++ thread

		if exit:  # can exit protocol
			print("Initiating CAN exit.")
			issueCommands.car.exitCAN()  # runs until we exit gracefully

		
	elif rc == 12:
		if 'ser' not in issueCommands.__dict__:
			issueCommands.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=5)

		steering = min(17, max(-17, steering))
		print(steering)
		commandSteering = "b " + str(1500+int(17.5*steering)) + " \n"
		print(commandSteering)         
		issueCommands.ser.write(commandSteering.encode())

		commandVelocity = "a " + str(1525 + int((velocity-40)/2)) + " \n"
		issueCommands.ser.write(commandVelocity.encode())
