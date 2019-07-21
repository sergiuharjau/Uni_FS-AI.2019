import cv2
import numpy as np

def findColour(openCVobject, greenDetection, cFlip, exitDetection=False) -> object:
	"""Function that takes path of an image and outputs a new file highlighting said colour.
	:param openCVobject: variable pointing to an openCVobject
	:param output: whether you want it saved to the file system as well or not
	"""
	image = openCVobject  # gcolour
   # blurred_image = cv2.GaussianBlur(image, (5, 5), 0)  # blurred to remove noise

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# RED COLOUR BOUNDARIES
	#red_lower = np.array([0, 120, 120])  # pair of least red and most red on the hsv map
	#red_upper = np.array([7, 255, 255])
	#wrap_around_lower = np.array([170, 120, 60])  # still need tweaking, doesnt pick up very light reds
	#wrap_around_upper = np.array([180, 255, 255])  # Could draw box around item to remedy this?

	# YELLOW COLOUR BOUNDARIES
	yellow_lower = np.array([20, 70, 70])
	yellow_upper = np.array([35, 255, 255])

	red_lower = np.array([108, 55, 55]) #blue in hsv
	red_upper = np.array([132, 255, 255]) #blue in hsv

	#red detection
	maskRed = cv2.inRange(hsv, red_lower, red_upper)
	#additional_mask = cv2.inRange(hsv, wrap_around_lower, wrap_around_upper)
	#maskRed += additional_mask

	#yellow detection
	maskYellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

	stopFlag = False
	if greenDetection:
		green = cv2.inRange(hsv, np.array([45, 100, 60]), np.array([65, 255, 255]))
		print(len(np.where(green==255)[0]))
		#input()
		if exitDetection:
			try:
				stopFlag = np.where(green==255)[0]
			except:
				stopFlag = "Out of sight"
		elif len(np.where(green==255)[0]) > 500:
			stopFlag = True

	if cFlip:
		return maskYellow, maskRed, stopFlag
	else:
		return maskRed, maskYellow, stopFlag

if __name__ == "__main__":

	image = cv2.imread("../test/hsv_map.png")

	#input()
	r, y, stop= findColour(image, False)
	print(stop)
	if stop:
		print("Attention, pedestrian!")
	cv2.imshow("image", image)
	#cv2.imshow("red", cv2.bitwise_and(image, image, mask=r))
	#cv2.imshow("yellow", cv2.bitwise_and(image, image, mask=y))
	cv2.imshow("blue", cv2.bitwise_and(image,image, mask=r))
	cv2.waitKey(0)
