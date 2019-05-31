
import cv2
from colourDetection import findColour
import numpy as np
import time
from bisect import bisect_right
import itertools

def thresh(a, threshmin=None, threshmax=None, newval=0):
	a = np.ma.array(a, copy=True)
	mask = np.zeros(a.shape, dtype=bool)
	if threshmin is not None:
		mask |= (a < threshmin).filled(False)

	if threshmax is not None:
		mask |= (a > threshmax).filled(False)

	a[mask] = newval
	return a

def findMin(bigList, k):
	flattened_list  = list(itertools.chain(*bigList))
	flattened_list.sort()
	try:
		min_val = flattened_list[bisect_right(flattened_list,k)]
	except:
		min_val = 0	
	return min_val

def findFirstPlane(red, yellow, depth):
	"""Inputs are red, yellow and depth mask."""
	
	conesDepth = cv2.bitwise_and(depth, depth, mask=red+yellow)
	planeDistance = findMin(conesDepth, 0.7)

	markedPixels = np.zeros((len(depth), len(depth[0])), dtype=np.int8)
	
	if planeDistance == 0:
		return markedPixels #empty array 
	#print("First plane distance: ", planeDistance)

	for hz in range(len(conesDepth)):
		for px in range(len(conesDepth[hz])):
			#print(conesDepth[hz][px], end=" ")
			if conesDepth[hz][px]: #ignore 0's
				if conesDepth[hz][px] - planeDistance < 0.5:
					markedPixels[hz][px] = 255
	#	#print()

	firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
	firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

	#cv2.imshow("firstRed", firstRed)
	#cv2.imshow("firstYellow", firstYellow)

	return firstRed, firstYellow


def findLineMarkers(red, yellow):
	redMarker=None
	exitLoop = False
	for hz in range(len(red)):
		for px in range(len(red[hz])):
			if red[hz][px]:
				redMarker = (px,hz)
				#print("Red exists ", redMarker)
				exitLoop = True
				break
		if exitLoop:
			break
	exitLoop = False
	yellowMarker=None
	for hz in range(len(yellow)):
		for px in reversed(range(len(yellow[hz]))):
			if yellow[hz][px]:
				yellowMarker = (px,hz)
				#print("Yellow exists", yellowMarker)
				exitLoop = True
			if exitLoop:
				break
		if exitLoop:
			break

	if redMarker and yellowMarker: 
		#print("Both exist")
		return redMarker, yellowMarker
	return (0,0), (0,0)
if __name__ == "__main__":
	pass
	#a = [0.7,0.8,0.9, 0, 3, 5, 0.75]
	#print(threshold(a, 0.7, 1, int(0))) 
	#image = cv2.imread("normal.png")
	#red, yellow = findColour(image)
	#processMasks(red, yellow, [])
