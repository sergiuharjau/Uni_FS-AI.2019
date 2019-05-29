import sys
import numpy as np
import pyzed.sl as sl
import cv2
import time
from colourDetection import findColour
from maskProcessing import findFirstPlane, findLineMarkers

def displayMeasure(measureMap):
	"""Displays a 2d array onto the terminal based on measure data"""
	for hz in range(len(measureMap)):
		for vt in measureMap[hz]:
			pixel = str(round(vt,3)) + " "*(6-len(str(round(vt,3))))
			print(pixel, end = ", ")
		print()

def main(visual = False) :
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
	width = image_size.width/3
	height = image_size.height/3

	# Declare sl.Mat matrices
	image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
	depth_image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
	# Create a sl.Mat with float type (32-bit)
	depth_data_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_32F_C1)
	count = 0
	startTime = time.time()
	for amount in range(200):
		err = zed.grab(runtime)
		if err == sl.ERROR_CODE.SUCCESS :
			# Retrieve the left image, depth image in specified dimensions
			zed.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(width), int(height))
			zed.retrieve_measure(depth_data_zed, sl.MEASURE.MEASURE_DEPTH)

			image_ocv = image_zed.get_data()

			depth_data_ocv = depth_data_zed.get_data()

			resizedDepth = cv2.resize(depth_data_ocv, dsize=(int(width),int(height)), interpolation = cv2.INTER_CUBIC)
			
			maskRed, maskYellow = findColour(image_ocv)
			
			combinedMask = maskRed + maskYellow

			fRed, fYellow = findFirstPlane(maskRed[230:300], maskYellow[230:300], resizedDepth[230:300])


			cv2.waitKey(10)
			print(amount)

			if visual:
				zed.retrieve_image(depth_image_zed, sl.VIEW.VIEW_DEPTH, sl.MEM.MEM_CPU, int(width), int(height))
				depth_image_ocv = depth_image_zed.get_data()

				combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=combinedMask)
				redImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed)
				yellowImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskYellow)
				combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=combinedMask)
				conesDepth = cv2.bitwise_and(depth_image_ocv, depth_image_ocv, mask=combinedMask)
				cv2.imshow('red', redImage)
				cv2.imshow('yellow', yellowImage)
				cv2.imshow('combined', combinedImage)
				cv2.imshow('conesDepth', conesDepth)

				redLine, yellowLine = findLineMarkers(fRed, fYellow)
				cv2.line(image_ocv[230:300], redLine, yellowLine, (0,255,0), 10)
				cv2.imshow("line image", image_ocv)

				cv2.waitKey(10)
			

		else:
			count += 1
			print(err)
			time.sleep(0.1)

	zed.close()
	print("Amount of skipped frames: ", count)
	print("Seconds it took: ", time.time()-startTime )
	print("\nFINISH")

if __name__ == "__main__":
	main(True) 
