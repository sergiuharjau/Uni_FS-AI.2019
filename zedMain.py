import sys
import numpy as np
import pyzed.sl as sl
import cv2
import time
from colourDetection import findColour

def displayMeasure(measureMap):
	"""Displays a 2d array onto the terminal based on measure data"""
	for hz in range(len(measureMap)):
		for vt in measureMap[hz]:
			pixel = str(round(vt,3)) + " "*(6-len(str(round(vt,3))))
			print(pixel, end = ", ")
		print()

def main(output = False) :
	"""Output parameter determines whether we save to the filesystem or not."""

	# Create a ZED camera object
	zed = sl.Camera()

	# Set configuration parameters
	init = sl.InitParameters()
	init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
	init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_PERFORMANCE
	init.coordinate_units = sl.UNIT.UNIT_METER
	if len(sys.argv) >= 2 :
		init.svo_input_filename = sys.argv[1]

	# Open the camera
	err = zed.open(init)
	if err != sl.ERROR_CODE.SUCCESS :
		print(repr(err))
		zed.close()
		exit(1)

	# Set runtime parameters after opening the camera
	runtime = sl.RuntimeParameters()
	runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD

	image_size = zed.get_resolution()
	width = image_size.width
	height = image_size.height

	# Declare sl.Mat matrices
	image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
	depth_image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
	# Create a sl.Mat with float type (32-bit)
	depth_data_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_32F_C1)
	count = 0
	for amount in range(5):
		err = zed.grab(runtime)
		if err == sl.ERROR_CODE.SUCCESS :
			# Retrieve the left image, depth image in specified dimensions
			zed.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(width), int(height))
			zed.retrieve_measure(depth_data_zed, sl.MEASURE.MEASURE_DEPTH)

			image_ocv = image_zed.get_data()
			depth_data_ocv = depth_data_zed.get_data()

			resizedDepth = cv2.resize(depth_data_ocv, dsize=(17,10), interpolation = cv2.INTER_CUBIC)
			
			maskRed, maskYellow = findColour(image_ocv)			
			
			combinedMask = maskRed + maskYellow

			print(amount)

			if output:
				#displayMeasure(resizedDepth)
				zed.retrieve_image(depth_image_zed, sl.VIEW.VIEW_DEPTH, sl.MEM.MEM_CPU, int(width), int(height))
				depth_image_ocv = depth_image_zed.get_data()
				cv2.imwrite("normal.png", image_ocv)
				cv2.imwrite("depth.png", depth_image_ocv)

				redImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed) 
				yellowImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskYellow)
				combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=combinedMask)
				conesDepth = cv2.bitwise_and(depth_image_ocv, depth_image_ocv, mask=combinedMask)
				cv2.imwrite('red.jpg', redImage)
				cv2.imwrite('yellow.jpg', yellowImage)
				cv2.imwrite('combined.jpg', combinedImage)
				cv2.imwrite('conesDepth.jpg', conesDepth)

		else:
			count += 1
			print(err)
			time.sleep(0.1)

	zed.close()
	print("Amount of skipped frames:", count)
	print("\nFINISH")

if __name__ == "__main__":
	main(True) 
