
import cv2
from colourDetection import findColour
import numpy as np
import time
from bisect import bisect_right
import itertools

def findMin(bigList, k):
	flattened_list  = list(itertools.chain(*bigList))
	flattened_list.sort()
	try:
		min_val = flattened_list[bisect_right(flattened_list,k)]
	except:
		min_val = 0	
	return min_val

def processMasks(red, yellow, depth):
	"""Inputs are red, yellow and depth mask."""

	conesDepth = cv2.bitwise_and(depth, depth, mask=red+yellow)
	planeDistance = findMin(conesDepth, 0.7)
	#print("First plane distance: ", planeDistance)

	if planeDistance == 0:
		return None

	markedPixels = np.zeros((len(depth), len(depth[0])), dtype=np.int8) 	
	count = 0
	for hz in range(len(conesDepth)):
		for px in range(len(conesDepth[hz])):
			#print(conesDepth[hz][px], end=" ")
			if abs(conesDepth[hz][px] - planeDistance) < 0.3:
				markedPixels[hz][px] = 255
				count+=1
		#print()

	firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
	firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)
	"""
	print("Marked total: ", count)	
	countAgain = 0 
	for element in firstRed:
		for pixel in element:
			if pixel != 0:
				countAgain += 1 
	print("Actual red: ", countAgain)


	countAgain = 0 
	for element in firstRed:
		for pixel in element:
			if pixel != 0:
				countAgain += 1 
	print("Actual yellow: ", countAgain)"""
	cv2.imshow("red", red)
	cv2.imshow("yellow", yellow)
	cv2.imshow("depth", conesDepth)
	cv2.imshow("firstRed", firstRed)
	cv2.imshow("firstYellow", firstYellow)
	cv2.waitKey(10)
	#input()	

if __name__ == "__main__":
	pass
	#image = cv2.imread("normal.png")
	#red, yellow = findColour(image)
	#processMasks(red, yellow, [])
