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

global width
width = 1280
global height
height = 720
global pixelStrip
pixelStrip = 30
global startFrom
startFrom = 270


def issueCommands(steering, velocity, exit, lastCommandTime=0.025):
	if 'car' not in issueCommands.__dict__:
		issueCommands.car = fspycan_ext.Car("can0")
		issueCommands.setup = True
		print("Initiating CAN setup.")
		
	while issueCommands.setup: #can setup protocol
		setup = issueCommands.car.setupCAN()
		time.sleep(0.025) # >=5ms, <50ms

	issueCommands.car.set_steering_velocity(int(steering), int(velocity))
	issueCommands.car.commandsLoop()

	if lastCommandTime < 0.005:
		print("Sending commands too fast. Waiting 10ms")
		time.sleep(0.01)
	elif lastCommandTime > 0.05:
		print("Sent command too late.\n Stop the program.")
		#issueCommands.setup = True

	if exit: #can exit protocol
		print("Initiating CAN exit.")
		while issueCommands.car.exitCAN():
			time.sleep(0.025)
	
def calculateCenter(target):

	if 'pastCom' not in calculateCenter.__dict__:
		calculateCenter.pastCom = 0

	newCom = target - int(width/2) #offset from center of image

	if newCom != -int(width/2): #when we have a correct reading
		
		final =  calculateCenter.pastCom + (newCom-calculateCenter.pastCom) / 2
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

	if len(sys.argv) > 1 :
		visual = True
		if len(sys.argv) > 2:
			green = True

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
		while True:   #for amount in range(framesToDo):
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
					issueCommands((reading/15)*-1, 60, False, time.time()-lastCommand)
					lastCommand = time.time()

				t.join()
				#print("Frames left: ", framesToDo-amount)
		
			else:
				skipped += 1
				#print(err)
				time.sleep(0.001)
	except KeyboardInterrupt:
		issueCommands(0,0,True) #initiates the exit protocol
		zed.close()

	issueCommands(0,0,True)
	zed.close()
	print("Amount of skipped frames: ", skipped)
	print("Seconds it took: ", time.time()-startTime )
	print("Actual framerate: ", (framesToDo-skipped)/(time.time()-startTime-(skipped*0.001)))
	print("\nFINISH")

if __name__ == "__main__":
	main() 	
