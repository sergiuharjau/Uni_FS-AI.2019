import cv2
import numpy as np


def findColour(openCVobject, greenDetection) -> object:
	"""Function that takes path of an image and outputs a new file highlighting said colour.
	:param openCVobject: variable pointing to an openCVobject 
	:param output: whether you want it saved to the file system as well or not 
	"""

	image = openCVobject  # gcolour
   # blurred_image = cv2.GaussianBlur(image, (5, 5), 0)  # blurred to remove noise
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# RED COLOUR BOUNDARIES
	#red_lower = np.array([0, 150, 60])  # pair of least red and most red on the hsv map
	#red_upper = np.array([7, 255, 359])
	#wrap_around_lower = np.array([170, 150, 60])  # still need tweaking, doesnt pick up very light reds
	#wrap_around_upper = np.array([180, 255, 359])  # Could draw box around item to remedy this?

	# YELLOW COLOUR BOUNDARIES
	yellow_lower = np.array([20, 100, 100])
	yellow_upper = np.array([35, 255, 255])
	
	red_lower = np.array([100, 100, 100]) #actually blue now
	red_upper = np.array([140, 255, 255]) #actually blue now

	#red detection
	maskRed = cv2.inRange(hsv, red_lower, red_upper)
	#additional_mask = cv2.inRange(hsv, wrap_around_lower, wrap_around_upper)
	#maskRed += additional_mask

	#yellow detection 
	maskYellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

	#green detection
	stopFlag = False
	if greenDetection:
		green = cv2.inRange(hsv, np.array([45, 100, 60]), np.array([65, 255, 255]))
		print(len(np.where(green==255)[0]))
		#input()
		if len(np.where(green==255)[0]) > 500:
			stopFlag = True

	return maskRed, maskYellow, stopFlag

if __name__ == "__main__":
	
	image = cv2.imread("hsv.jpg")

	#input()
	r, y, stop = findColour(image, False)
	print(stop)
	if stop:
		print("Attention, pedestrian!")
	cv2.imshow("image", image)
	cv2.imshow("red", cv2.bitwise_and(image, image, mask=r))
	cv2.imshow("yellow", cv2.bitwise_and(image, image, mask=y))
	cv2.waitKey(0)
