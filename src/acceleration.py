from cmds import issueCommands
from gps import GPS 
from geopy import distance

if __name__ == "__main__":
	gps = GPS(logging=False)

	while gps.coords() == (0,0):
		print("Awaiting gps lock.")

	startingPos = gps.coords()
	print("GPS lock acquired, starting position: ", startingPos)
	
	distanceAway = 0
	while distanceAway < 100:
		coordinates = gps.coords()
		if coordinates == (0,0):
			print("Lost GPS. Stopping until we regain")
			issueCommands(0,0)
		distanceAway = (distance.distance(gps.coords(), startingPos)).m
		print("Distance away from destination: ", distanceAway)
		issueCommands(0,150)
		time.sleep(0.2)

	print("Reached destination, stopping.")
	issueCommands(0,0,True)
    gps.stop()
