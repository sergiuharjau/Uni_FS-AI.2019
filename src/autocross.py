import time
from imProc import imProcessing
from cmds import calculateReading, issueCommands
from globals import pixelStrip, startFrom, logInitial
import cv2
import logging
#from gps import GPS
#from geopy import distance

def mainProgram(visual, rc, cFlip, ic):
# //Used when stopping is needed.
#     gps = GPS()
#     while gps.getGPS(force=1) == (0,0):
#         print("Awaiting GPS lock.")
#         time.sleep(1) 
# 
    calculateReading.pastCom = 0  # in case we don't see cones straight away
    startTime = time.time()
    # lapCounter = 0 
    # timeMarker = time.time()
    # setStart = True

    original_image, depth = ic.latest()
    
    depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
    image_ocv = original_image[startFrom:startFrom + pixelStrip]

    steering, velocity = imProcessing(image_ocv, depth_data_ocv, visual, original_image, cFlip)

#   steering = min(19, max(-19, steering))
#Uncomment to cap steering to a certain boundary
# """ //To be used when counting laps
#             if setStart:
#                 velocity -= 30
#                 if time.time()-startTime > 12:
#                     startingPos = gps.getGPS()
#                     setStart = False
#                     gps.coords = (-1,-1)
#             if time.time() - timeMarker > 70:
#                velocity -= 50
#             if time.time()-timeMarker > 75: #only checks 30s after we've passed the starting point
#                 gps.getGPS(timeBound=2)
#                 if distance.distance(gps.getCoords(), startingPos).m < 7: #5m within the finish line
#                     lapCounter += 1
#                     timeMarker = time.time() #resets the time marker
#                     if lapCounter == 10: #change to 10 in the future 
#                         raise KeyboardInterrupt
# """   
       
    ic.pub(steering, velocity)

    print("Steering: %d, Velocity: %d" % (steering, velocity))
    logging.info("Steering: %d, Velocity: %d", steering, velocity)

    #time.sleep(1) ??

    print("Seconds it took: ", time.time() - startTime)
    print("Actual framerate: ", 1 / (time.time() - startTime))

    logging.info("\nTotal seconds: %d", time.time()-startTime)
    logging.info("Framerate: %d", 1 / (time.time() - startTime))