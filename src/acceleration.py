from cmds import issueCommands
from gps import GPS 
from geopy import distance
import time

if __name__ == "__main__":
#	gps = GPS(logging=False)

#	while gps.getCoords() == (0,0):
#		print(gps.getCoords())
#		print("Awaiting gps lock.")

#	startingPos = gps.getCoords()
#	print("GPS lock acquired, starting position: ", startingPos)
	issueCommands(0,0) # until it connects

	startLeft = issueCommands.car.get_left_pulse()
	startRight = issueCommands.car.get_right_pulse()
	start = time.time()
	distanceAway = 0
	while distanceAway < 50:
#		coordinates = gps.coords()
#		if coordinates == (0,0):
#			print("Lost GPS. Stopping until we regain")
#			issueCommands(0,0)
#		distanceAway = (distance.distance(gps.getCoords(), startingPos)).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(0,100)
		time.sleep(2)
	print("Total time: ", time.time()-start)
	print("Total left pulse: ", issueCommands.car.get_left_pulse() - startLeft)
	print("Total right pulse: ", issueCommands.car.get_right_pulse() - startRight)
	print("Reached destination, stopping.")

	issueCommands(0,0,True)
#	gps.stop()
