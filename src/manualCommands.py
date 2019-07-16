from cmds import issueCommands
from globals import carVelocity

if __name__ == "__main__":
	while True:
		try: 
			steering = int(input("Steering: "))
			issueCommands(steering, carVelocity)
		except KeyboardInterrupt:
			issueCommands(0,0)
			time.sleep(4)
			issueCommands(0,0, True)