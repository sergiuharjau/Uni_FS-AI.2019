import sys
import numpy as np
import pyzed.sl as sl
import cv2
import time
from colourDetection import findColour
from maskProcessing import findGates, findLineMarkers
import threading

sys.path.insert(0, '../fspycan/lib/')
import fspycan_ext

def issueCommands(steering, velocity):
	if 'car' not in issueCommands.__dict__:
		issueCommands.car = fspycan_ext.Car("can0")
	issueCommands.car.set_steering_velocity(int(steering), int(velocity))
	issueCommands.car.loop()

	
def targetProcessing(target):
	"""Deals with calculations based on target. Returns an average once every 15 frames."""

	if 'total' not in targetProcessing.__dict__:
		targetProcessing.total = 0
	if 'frameCounter' not in targetProcessing.__dict__:
		targetProcessing.frameCounter = 0

	offset = target[0] - 640 #offset from center of image

	if offset != -640: #when we have a correct reading
		targetProcessing.total += offset
		targetProcessing.frameCounter +=1

	if targetProcessing.frameCounter == 15:
		average = int(targetProcessing.total / targetProcessing.frameCounter)
		targetProcessing.total = 0
		targetProcessing.frameCounter = 0

		return average

def imCapt(zed, lock):
	"""Used for parallelised image and depth campturing."""

	print("Different thread started")
	start = time.time()

	zed.retrieve_image(imCapt.image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU)
	zed.retrieve_measure(imCapt.depth_data_zed, sl.MEASURE.MEASURE_DEPTH, sl.MEM.MEM_CPU)

	print("Different thread took: ", time.time()-start)

def imProcessing(image_ocv, depth_data_ocv, visual, zed, original_image):
	print("Started processing")
	processing = time.time()

	maskRed, maskYellow = findColour(image_ocv)
	combinedMask = maskRed + maskYellow
	print("Processed colour")
	fRed, fYellow, secRed, secYellow = findGates(maskRed, maskYellow, depth_data_ocv)
			 #finds the masks for the first red/yellow cones
	print("Processed gate depth")
	#FirstGate
	redLine1, yellowLine1 = findLineMarkers(fRed, fYellow) 
	target1 = (int((yellowLine1[0] + redLine1[0])/2), int((yellowLine1[1] + redLine1[1])/2))
		#the center of the first gate
	#SecondGate
	redLine2, yellowLine2 = findLineMarkers(secRed, secYellow)			
	target2 = (int((yellowLine2[0] + redLine2[0])/2), int((yellowLine2[1] + redLine2[1])/2))
	print("Processed line markers")
	reading = targetProcessing(target1) #averages a reading every 15 frames

	print("All of processing took: ", time.time()-processing)
		
	if visual:
		depth_image_zed = sl.Mat(1280, 720, sl.MAT_TYPE.MAT_TYPE_8U_C4)
		zed.retrieve_image(depth_image_zed, sl.VIEW.VIEW_DEPTH, sl.MEM.MEM_CPU)
		depth_image_ocv = depth_image_zed.get_data()

		combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=combinedMask)
		redImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed)
		yellowImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskYellow)
		combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=combinedMask)

		cv2.imshow("firstGate", fRed + fYellow)
		cv2.imshow("secondGate", secRed + secYellow)
		cv2.imshow('colour data', combinedImage)
		cv2.imshow('full depth', depth_image_ocv)


	#FirstGate
		if redLine1[0]:
			cv2.line(original_image[270:300], redLine1, yellowLine1, (0,255,0), 10)
			cv2.circle(original_image[270:300], target1, 5, (0,0,255), 4)
			center = (int(1280/2), 0)
			cv2.line(original_image[270:300], target1, center, (255,0,0), 2)
	#SecondGate
		if redLine2[0]:
			cv2.line(original_image[270:300], redLine2, yellowLine2, (255,0,0), 5)
			cv2.circle(original_image[270:300], target2, 5, (255,0,255), 2)

		cv2.imshow("image", original_image)
		cv2.imshow("cropped", image_ocv)

		cv2.waitKey(10)

	return reading

def main(visual = False) :

	# Create a ZED camera object
	zed = sl.Camera()

	# Set configuration parameters
	init = sl.InitParameters()
	init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
	init.camera_fps = 60 # Set max fps at 100

	init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA
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

	width = 1280
	height = 720

	# Declare sl.Mat matrices
	imCapt.image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
	# Create a sl.Mat with float type (32-bit)
	imCapt.depth_data_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_32F_C1)

	skipped = 0
	startTime = time.time()
	framesToDo = 500
	lock = threading.Lock()

	for amount in range(framesToDo):
		start = time.time()
		err = zed.grab(runtime)
		if err == sl.ERROR_CODE.SUCCESS:
			# Retrieve the left image, depth image in specified dimensions
			print("New frame")

			transcribing = time.time()
			original_image = imCapt.image_zed.get_data()
			depth_data_ocv = imCapt.depth_data_zed.get_data()[270:300]
			image_ocv = original_image[270:300]

			print("Transcribing took: ", time.time()-transcribing)

			print("Starting threading")
			t = threading.Thread(target=imCapt, args=(zed,lock))
			t.start()
			print("Thread started, time elapsed from start: ", time.time()-start)

			reading = imProcessing(image_ocv, depth_data_ocv, visual, zed, original_image)

			if reading:
				print("Camera: ", reading)
				issueCommands((reading/20)*-1,50)

			#print("Frames left: ", framesToDo-amount)
			
			join = time.time()
			t.join()
			print("Joining took:", time.time()-join)
			print("Whole frame took: ", time.time() - start)


			
		else:
			skipped += 1
			print(err)
			time.sleep(0.001)

	zed.close()
	print("Amount of skipped frames: ", skipped)
	print("Seconds it took: ", time.time()-startTime )
	print("Actual framerate: ", (framesToDo-skipped)/(time.time()-startTime))
	print("\nFINISH")

if __name__ == "__main__":
	main(False) 
