import sys
from colour import findColour
from maskProc import findGates, findLineMarkers
from capturing import ImageCap
from imProc import imProcessing
from cmds import findLineMarkers, calculateReading, issueCommands

width = 1280
height = 720 #consider dropping camera res as well

pixelStrip = 100
startFrom = 300

def main(visual=False, green=False) :

	calculateReading.pastCom = 0 #in case we don't see cones straight away
	if len(sys.argv) > 1 :
		visual = sys.argv[1]
		if len(sys.argv) > 2:
			green = sys.argv[2]
	
	ic = ImageCap() #ImCapt() #initializes zed object

	startTime = time.time()
	#framesToDo = 200

	try:
		while True: #for amount in range(framesToDo):

			image, depth = ic.latest() 

			original_image = image.get_data()
			depth_data_ocv = depth.get_data()[startFrom:startFrom+pixelStrip]
			image_ocv = original_image[startFrom:startFrom+pixelStrip]

			reading = imProcessing(image_ocv, depth_data_ocv, visual, original_image, green)
			ic.clear() #works faster for performance reasons

			if reading:
				print("Camera: ", reading)
				if not visual:
					issueCommands((reading/steeringFactor)*-1, carVelocity, False, visual)
			else:
				print("No camera reading.")
				if not visual:
					issueCommands((calculateCenter.pastCom/steeringFactor)*-1, carVelocity, False, visual)
			#print("Frames left: ", framesToDo-amount)

	except KeyboardInterrupt:
		issueCommands(0,0,True, visual) #initiates the exit protocol
		ic.stop()
		zed.close()
		quit()

	issueCommands(0,0,True, 0)
	ic.stop()
	zed.close()
	print("Amount of skipped frames: ", skipped)
	print("Seconds it took: ", time.time()-startTime )
	print("Actual framerate: ", (framesToDo-skipped)/(time.time()-startTime-(skipped*0.001)))
	print("\nFINISH")

if __name__ == "__main__":
	main() 	
