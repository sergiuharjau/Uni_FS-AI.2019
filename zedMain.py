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

global width; width = 1280
global height; height = 720 #consider dropping camera res as well

global pixelStrip; pixelStrip = 100
global startFrom; startFrom = 300

global steeringFactor; steeringFactor = 10 #divide PixelValue by this number
global newComOffset; newComOffset = 15 #divide newCom by this number=======global width

global carVelocity; carVelocity = 70


def issueCommands(steering, velocity, exit, lastCommandTime=0.025):
	if 'car' not in issueCommands.__dict__: #only runs once
		issueCommands.car = fspycan_ext.Car("can0")
		issueCommands.car.init()
		print("Initiating CAN setup.")
		issueCommands.car.setupCAN() #function runs until we finish setup
		print("Setup finished gracefully") 

	#sorted((-24, steering, 24))[1]

	issueCommands.car.set_steering_velocity(int(steering), int(velocity))
			#we only set the steering here, the loop runs on a different c++ thread


	if exit: #can exit protocol
		print("Initiating CAN exit.")
		issueCommands.car.set_steering_velocity(0,0)
		time.sleep(2)
		issueCommands.car.exitCAN() #runs until we exit gracefully

def calculateCenter(target):

	newCom = target - int(width/2) #offset from center of image

	if newCom != -int(width/2): #when we have a correct reading
		final =  calculateCenter.pastCom + (newCom-calculateCenter.pastCom) / newComOffset
		calculateCenter.pastCom = final

		return round(final)

def imCapt(zed):
	"""Used for parallelised image and depth campturing."""

	zed.retrieve_image(imCapt.image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU)
	zed.retrieve_measure(imCapt.depth_data_zed, sl.MEASURE.MEASURE_DEPTH, sl.MEM.MEM_CPU)


def imProcessing(image_ocv, depth_data_ocv, visual=False, original_image=None, green=False):

	maskRed, maskYellow, stop = findColour(image_ocv, green)

	if stop:
		print("Attention, pedestrian!")
		quit()

	findGates(maskRed, maskYellow, depth_data_ocv, True, 0.7)
			 #finds the masks for the first red/yellow cones
	targetList = []
	i = 0
	for gate in findGates.result:
		fRed = gate[0]
		fYellow = gate[1]
		redLine, yellowLine = findLineMarkers(fRed, fYellow, i, visual) 
		if redLine and yellowLine: #only on correct readings
			target = (int((yellowLine[0] + redLine[0])/2), int((yellowLine[1] + redLine[1])/2))
			targetList.append(target)
			targetList.append(redLine)
			targetList.append(yellowLine)
		i+=1

	reading = None
	if len(targetList):
		reading = calculateCenter(targetList[0][0]) #averages a reading every 15 frames

	if visual:
		if len(targetList):
			cv2.circle(original_image[startFrom:startFrom+pixelStrip], targetList[0], 5, (255,0,0), 4)
			cv2.line(original_image[startFrom:startFrom+pixelStrip], targetList[1], targetList[2], (0,255,0), 10)
			if len(targetList)>3:
				cv2.circle(original_image[startFrom:startFrom+pixelStrip], targetList[3], 5, (0,0,255), 4)
				cv2.line(original_image[startFrom:startFrom+pixelStrip], targetList[4], targetList[5], (0,0,0), 10)

		redImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed)
		yellowImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskYellow)
		combinedImage = cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed+maskYellow)

		cv2.imshow('colour data', combinedImage)
		cv2.imshow('full depth', depth_data_ocv)

		cv2.imshow("image", original_image)
		cv2.imshow("cropped", image_ocv)

		cv2.waitKey(10)

	return reading

def main(visual=False, green=False) :

	# Create a ZED camera object
	zed = sl.Camera()
	
	# Set configuration parameters
	init = sl.InitParameters()
	init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
	init.camera_fps = 60 # Set max fps at 100

	init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA
	init.coordinate_units = sl.UNIT.UNIT_METER
	
	calculateCenter.pastCom = 0
	lastCommand = 0

	if len(sys.argv) > 1 :
		visual = sys.argv[1]
		if len(sys.argv) > 2:
			green = sys.argv[2]

	# Open the camera
	err = zed.open(init)
	if err != sl.ERROR_CODE.SUCCESS :
		print(repr(err))
		zed.close()
		exit(1)

	# Set runtime parameters after opening the camera
	runtime = sl.RuntimeParameters()
	runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD

	# Declare sl.Mat matrices
	imCapt.image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
	# Create a sl.Mat with float type (32-bit)
	imCapt.depth_data_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_32F_C1)

	skipped = 0
	startTime = time.time()
	#framesToDo = 200
	try:
		while True: #for amount in range(framesToDo):
			err = zed.grab(runtime)
			if err == sl.ERROR_CODE.SUCCESS:
				# Retrieve the left image, depth image in specified dimensions

				original_image = imCapt.image_zed.get_data()
				depth_data_ocv = imCapt.depth_data_zed.get_data()[startFrom:startFrom+pixelStrip]
				image_ocv = original_image[startFrom:startFrom+pixelStrip]

				t = threading.Thread(target=imCapt, args=(zed,))

				reading = imProcessing(image_ocv, depth_data_ocv, visual, original_image, green)
				t.start() #works faster for performance reasons

				if reading:
					print("Camera: ", reading)
					if not visual:
						issueCommands((reading/steeringFactor)*-1, carVelocity, False, time.time()-lastCommand)
				else:
					print("No camera reading.")
					if not visual:
						issueCommands((calculateCenter.pastCom/steeringFactor)*-1, carVelocity, False, time.time()-lastCommand  )
				lastCommand = time.time()
				t.join()
				#print("Frames left: ", framesToDo-amount)

			else:
				skipped += 1
				#print(err)
				time.sleep(0.001)
	except KeyboardInterrupt:
		if not visual:
			issueCommands(0,0,True, 0) #initiates the exit protocol
		zed.close()
		quit()

	issueCommands(0,0,True, 0)
	zed.close()
	print("Amount of skipped frames: ", skipped)
	print("Seconds it took: ", time.time()-startTime )
	print("Actual framerate: ", (framesToDo-skipped)/(time.time()-startTime-(skipped*0.001)))
	print("\nFINISH")

if __name__ == "__main__":
	main() 	
