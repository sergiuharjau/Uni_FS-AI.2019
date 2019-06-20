
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
	maxFirstGate = planeDistance + 0.2
	markedPixels = threshold(conesDepth, planeDistance, maxFirstGate, 0)
		#only keep pixels in the desired threshold

	markedPixels[markedPixels>1] = 1 #if ever an element is bigger than 1, make it 1
	markedPixels = (markedPixels.round()* 255).astype(np.uint8) #transform to desired format

	firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
	firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

	findGates.result.append((firstRed,firstYellow))

	#findGates(red, yellow, depth, False, maxFirstGate+1)
	#re-Add in the future to allow multiple gate processing

def findLineMarkers(red, yellow, i, visual):

	redIndex = np.where(red==255)
	if len(redIndex) == 0:
		redMarker = (-250,0)
	else:
		redMarker = (redIndex[1][0], redIndex[0][0])

	yellowIndex = np.where(yellow==255)
	if len(yellowIndex) == 0:
		yellowMarker = (1280+250,0)
	else:
		yellowMarker = (yellowIndex[1][0], yellowIndex[0][0])

	if visual:
		for x in range(i+1):
			cv2.imshow("gate " + str(i), red+yellow)

		for x in range(i+1, 5):
			cv2.destroyWindow("gate " + str(x))

	return redMarker, yellowMarker


if __name__ == "__main__":
	pass
