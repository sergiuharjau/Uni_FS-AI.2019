from cmds import issueCommands
from gps import GPS
import math

if __name__ == "__main__":


	gps = GPS(logging=False)

	while gps.getCoords() == (0,0):
		print("Awaiting gps lock.")

	startingPos = gps.coords()

	issueCommands(0,0)	

	distanceAway = 0
	while distanceAway < 15: #go forward 15m
		distanceAway = (distance.distance(gps.getCoords(), startingPos)).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(0,85)

	issueCommands(0,0) #connect to car

	startLeft = issueCommands.car.get_left_pulse() #should be 0
	startRight = issueCommands.car.get_right_pulse() #should be 0 
	carWidth = 1.17 #in meters
	fullCircleDistance = 2 * math.pi * carWidth

	tyreCirc = PI * 0.58
	fullCircleRotations = fullCircleDistance / tyreCirc
	fullCirclePulses = fullCircleRotations / 20
	flip = 1 

	try: 
		while True:
			##get odometry on wheels every frame
			leftPulse=issueCommands.car.get_left_pulse()
			rightPulse=issueCommands.car.get_right_pulse()

			tacoDiff = abs( (rightPulse-startRight) - (leftPulse-startLeft))

			issueCommands(flip*-24, 85) #go slightly right 
									#in the future this follows the blue cones

			if tacoDiff >= fullCirclePulses: #when we complete a circle, flip it around
				flip = -1 
				startRight = rightPulse
				startLeft = leftPulse
	except KeyboardInterrupt:
		issueCommands(0,0, True)
		gps.stop()

		##calculate if we've done a full circle yet 