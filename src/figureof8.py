from cmds import issueCommands
from math import PI

if __name__ == "__main__":


	gps = GPS(logging=False)

	while gps.coords() == (0,0):
		print("Awaiting gps lock.")

	startingPos = gps.coords()

	issueCommands(0,0)	

	distanceAway = 0
	while distanceAway < 15: #go forward 15m
		distanceAway = (distance.distance(gps.coords(), startingPos)).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(0,85)

	issueCommands(0,0) #connect to car

	startLeft = issueCommands.car.get_left_pulse() #should be 0
	startRight = issueCommands.car.get_right_pulse() #should be 0 
	carWidth = #
	fullCircleDistance = 2 * PI * carWidth

	tyreCirc = PI * #diameter
	fullCircleRotations = fullCircleDistance / tyreCirc
	fullCirclePulses = fullCircleRotations / 20
	flip = 1 

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

		##calculate if we've done a full circle yet 
