
import cv2
from colourDetection import findColour
import numpy as np
import time
from bisect import bisect_right
import itertools

def threshold(a, threshmin=None, threshmax=None, newval=0):
	"""Returns an array with values bound between threshmin and threshmax """
	a = np.ma.array(a, copy=True)
	mask = np.zeros(a.shape, dtype=bool)
	if threshmin is not None:
		mask |= (a < threshmin).filled(False)

	if threshmax is not None:
		mask |= (a > threshmax).filled(False)

	a[mask] = newval

	return a

def findMin(bigList, k, firstPass=False):
	"""Returns the smallest value bigger than k in the array"""
	if 'flattened_list' not in findMin.__dict__ or firstPass:
		findMin.flattened_list  = list(itertools.chain(*bigList))	
		findMin.flattened_list.sort()

	try:
		min_val = findMin.flattened_list[bisect_right(findMin.flattened_list,k)]
	except:
		min_val = 0	

	return min_val

def findGates(red, yellow, depth, firstPass, gateDistance):
	"""Inputs are red, yellow and depth mask."""
	
	if firstPass:
		findGates.result = []

	conesDepth = cv2.bitwise_and(depth, depth, mask=red+yellow)
		#depth info just where the cones are

	planeDistance = findMin(conesDepth, gateDistance, firstPass)

	if planeDistance == 0: #no object in sight
		return None
			#empty pixels
	maxFirstGate = planeDistance + 1
	markedPixels = threshold(conesDepth, planeDistance, maxFirstGate, 0)
		#only keep pixels in the desired threshold

	markedPixels[markedPixels>1] = 1 #if ever an element is bigger than 1, make it 1
	markedPixels = (markedPixels.round()* 255).astype(np.uint8) #transform to desired format

	firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
	firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

	findGates.result.append((firstRed,firstYellow))

	#findGates(red, yellow, depth, False, maxFirstGate+1)
	#re-Add in the future to allow multiple gate processing


missedRed=0;missedYellow=0
def findLineMarkers(red, yellow, i, visual):

	redIndex = np.where(red==255)
	try: 
		redMarker = (redIndex[1][0], redIndex[0][0])
		missedRed = 0
	except:
		missed += 1
		if missedRed > 10:
			print("Can't see blue, turning left", (-100*missedRed+640)/2-640)
			return (-100 * missedRed, 0), (640, 0)
			#very far left red, middle yellow, turns left
		else:
			print("Missed Red: ", missedRed)
			redMarker = False

	yellowIndex = np.where(yellow==255)
	try:
		yellowMarker = (yellowIndex[1][0], yellowIndex[0][0])
		missedYellow = 0
	except:
		missedYellow += 1
		if missedYellow > 10:
			print("Can't see yellow, turning right", ((1280+100*missedYellow)-640)/2-640)
			return (640,0),(1280 + 100*missedYellow, 0)
				#middle red, very far right Yellow, turns right
		else:
			print("Missed Yellow: ", missedYellow)
			yellowMarker = False

	if visual:
		for x in range(i+1):
			cv2.imshow("gate " + str(i), red+yellow)

		for x in range(i+1, 5):
			cv2.destroyWindow("gate " + str(x))

	return redMarker, yellowMarker

if __name__ == "__main__":
	pass
