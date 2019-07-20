import cv2
from maskProc import findGates
from cmds import findLineMarkers, calculateReading
from colour import findColour
from globals import startFrom, pixelStrip, width, steeringFactor
import logging
import threading

def imProcessing(t, image_ocv, depth_data_ocv, visual=False, original_image=None, green=False, cFlip=0, eight=0, swapCircles=0, exitDetection=0, gates=3):

	logging.info("Started image processing.")
	maskRed, maskYellow, stop = findColour(image_ocv, green, cFlip, exitDetection)
	logging.info("Received colour data.")

	if stop == True:
		print("Attention, pedestrian!")
		raise KeyboardInterrupt
	logging.info("Finding gates.")
	findGates(maskRed, maskYellow, depth_data_ocv, True, 2, 12, gates) #just one gate
	logging.info("Started capturing thread.")
	t.start()
	# finds the masks for the first red/yellow cones
	gateDict = {}
	for i, gate in enumerate(findGates.result):
		fRed = gate[0]
		fYellow = gate[1]
		redLine, yellowLine = findLineMarkers(fRed, fYellow, i, visual)
		if redLine and yellowLine:  # only on correct readings
			target = (int((yellowLine[0] + redLine[0]) / 2),int((yellowLine[1] + redLine[1]) / 2))
			logging.info("Validated %d!", i)
			if eight and swapCircles: #yellow on the right is real, we fake red
				gateDict[i] = [target, (int(width/4),0), yellowLine]
			elif eight: #red on the left is real, we fake yellow
				gateDict[i] = [target, redLine, int(3 * width/4)]
			else: #normal behaviour
				gateDict[i] = [target, redLine, yellowLine]
		else:
			logging.info("Gate %d not valid.", i)

	steering, velocity =  calculateReading(gateDict)

	if visual:
		colours = [(0,255,0), (255,0,0), (0,0,0), (0,0,255)]
		for key in gateDict:
			cv2.line(original_image[startFrom:startFrom + pixelStrip], gateDict[key][1], gateDict[key][2], colours[key], 10)

		cv2.imshow('colour data', cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed + maskYellow))
		cv2.imshow("image", original_image)
		cv2.imshow("cropped", image_ocv)
		cv2.waitKey(10)

	logging.info("Finished image processing.")

	if exitDetection:
		if stop == "Out of sight":
			steering = 0 
		else:
			steering = (width/2 - stop) / steeringFactor
		velocity = 100

	return steering, velocity
