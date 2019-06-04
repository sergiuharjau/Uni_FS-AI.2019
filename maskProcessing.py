
import cv2
from colourDetection import findColour
import numpy as np
import time
from bisect import bisect_right
import itertools

def threshold(a, threshmin=None, threshmax=None, newval=0):

    a = np.ma.array(a, copy=True)
    mask = np.zeros(a.shape, dtype=bool)
    if threshmin is not None:
        mask |= (a < threshmin).filled(False)

    if threshmax is not None:
        mask |= (a > threshmax).filled(False)

    a[mask] = newval
    return a

def findMin(bigList, k, firstPass=False):
	if 'flattened_list' not in findMin.__dict__ or firstPass:
		findMin.flattened_list  = list(itertools.chain(*bigList))	
		findMin.flattened_list.sort()
		#print("New frame, changing sorted list")
	try:
		min_val = findMin.flattened_list[bisect_right(findMin.flattened_list,k)]
	except:
		min_val = 0	
	#print("Min_val: ",min_val)
	return min_val

def findGates(red, yellow, depth):
	"""Inputs are red, yellow and depth mask."""

	conesDepth = cv2.bitwise_and(depth, depth, mask=red+yellow)
		#depth info just where the cones are
#First Gate
	start = time.time()
	planeDistance = findMin(conesDepth, 0.7, True)
	#print("First findMin: ", time.time()-start)
	markedPixels = np.zeros((len(depth), len(depth[0])), dtype=np.int8)
	#print("Looking for first gate")

	if planeDistance == 0: #no object in sight
		#print("Didn't find first gate")
		#print("Total: ", time.time() - start)
		return markedPixels, markedPixels, markedPixels, markedPixels
			#empty pixels
	maxFirstGate = planeDistance + 0.5
	markedPixels = threshold(conesDepth, planeDistance, maxFirstGate, 0)
		#only keep pixels in the desired threshold

	markedPixels[markedPixels>1] = 1 #if ever an element is bigger than 1, make it 1
	markedPixels = (markedPixels.round()* 255).astype(np.uint8) #transform to desired format

	firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
	firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)
	#print("Found first gate in: ", time.time() - start)
	
#Second Gate
	#print("Looking for second gate")
	second = time.time()
	secondGate = findMin(conesDepth, maxFirstGate + 1, False)
	#print("Second findMin took: ", time.time()-second)
	markedPixels = np.zeros((len(depth), len(depth[0])), dtype=np.int8)

	if secondGate == 0:
		#print("Didnt find secondGate")
		#print("Total: ", time.time()-start)
		return firstRed, firstYellow, markedPixels, markedPixels
		#return what we have so far

	maxSecondGate = secondGate + 0.5
	markedPixels = threshold(conesDepth, secondGate, maxSecondGate, 0)
			#only keep pixels in the desired threshold

	markedPixels[markedPixels>1] = 1 #if ever an element is bigger than 1, make it 1
	markedPixels = (markedPixels.round()* 255).astype(np.uint8) #transform to desired format
	
	secondRed = cv2.bitwise_and(red, red, mask=markedPixels)
	secondYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

	#print("Found second gate: ", time.time()-second)
	#print("Total: ", time.time()-start)
	#input()
	return firstRed, firstYellow, secondRed, secondYellow


def findLineMarkers(red, yellow):

	try:
		redIndex = np.where(red==255)
		redMarker = (redIndex[1][0], redIndex[0][0])

		yellowIndex = np.where(yellow==255)
		yellowMarker = (yellowIndex[1][0], yellowIndex[0][0])
	except:
		return(0,0), (0,0)

	return redMarker, yellowMarker


if __name__ == "__main__":
	pass
