import logging

width = 1280
height = 720  # consider dropping camera res as well
	
pixelStrip = 100
startFrom = 330

steeringFactor = 17  # divide PixelValue by this number
newComOffset = 9  # divide newCom by this number

missedColourOffset = 15

carVelocity = 70
maxSpeedUp = 30

def logInitial():
	logging.info("Resolution: %d, %d", width, height)
	logging.info("Pixel slice: %d, starting at %d", pixelStrip, startFrom)
	logging.info("Steering factor: %d", steeringFactor)
	logging.info("Rolling average: %d", newComOffset)
	logging.info("Missed colour offset: %d", missedColourOffset)
	logging.info("Base velocity: %d", carVelocity)
	logging.info("Max speed up: %d\n\n", maxSpeedUp)
