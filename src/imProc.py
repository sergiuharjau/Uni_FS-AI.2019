import cv2
from maskProc import findGates
from cmds import findLineMarkers, calculateReading
from colour import findColour
from globals import *

def imProcessing(image_ocv, depth_data_ocv, visual=False, original_image=None, green=False):

	maskRed, maskYellow, stop = findColour(image_ocv, green)

	if stop:
		print("Attention, pedestrian!")
		issueCommands(0, 0, True, visual)
		sleep(2)
		zed.close()
		quit()

	findGates(maskRed, maskYellow, depth_data_ocv, True, 0.7)
			 #finds the masks for the first red/yellow cones
	targetList = []
	i = 0
	for gate in findGates.result:
		fRed = gate[0]
		fYellow = gate[1]
		redLine, yellowLine = findLineMarkers(fRed, fYellow, i, visual) 
		if redLine and yellowLine: #only on correct readings
			target = (int((yellowLine[0] + redLine[0])/2), int((yellowLine[1] + redLine[1])/2))
			targetList.append(target)
			targetList.append(redLine)
			targetList.append(yellowLine)
		i+=1

	reading = None
	if len(targetList):
		reading = calculateReading(targetList[0][0]) #averages a reading every 15 frames

	if visual:
		if len(targetList):
			cv2.circle(original_image[startFrom:startFrom+pixelStrip], targetList[0], 5, (255,0,0), 4)
			cv2.line(original_image[startFrom:startFrom+pixelStrip], targetList[1], targetList[2], (0,255,0), 10)
			if len(targetList)>3:
				cv2.circle(original_image[startFrom:startFrom+pixelStrip], targetList[3], 5, (0,0,255), 4)
				cv2.line(original_image[startFrom:startFrom+pixelStrip], targetList[4], targetList[5], (0,0,0), 10)

		redImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed)
		yellowImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskYellow)
		combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed+maskYellow)

		cv2.imshow('colour data', combinedImage)
		cv2.imshow('full depth', depth_data_ocv)

		cv2.imshow("image", original_image)
		cv2.imshow("cropped", image_ocv)

		cv2.waitKey(10)

	return reading
