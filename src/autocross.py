import time
import logging
from imProc import imProcessing
from globals import pixelStrip, startFrom
from cmds import calculateReading
#from gps import GPS
#from geopy import distance

def mainProgram(visual, cFlip, ros):
# //Used when stopping is needed.
#     while ros.gps == (0,0):
#         print("Awaiting GPS lock.")
#         time.sleep(1)
# 
    # lapCounter = 0
    # timeMarker = time.time()
    # setStart = True

    startTime = time.time()

    original_image, depth = ros.latestCamera()
    depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
    image_ocv = original_image[startFrom:startFrom + pixelStrip]

    steering, velocity = imProcessing(image_ocv, depth_data_ocv, visual, original_image, cFlip)
#   steering = min(19, max(-19, steering))
#Uncomment to cap steering to a certain boundary
    """ //To be used when counting laps
             if setStart:
                 velocity -= 30
                 if time.time()-startTime > 12:
                     startingPos = ros.gps                    
                      setStart = False
                      ros.gps
              if time.time() - timeMarker > 70:
                velocity -= 50
             if time.time()-timeMarker > 75: #only checks 30s after
                 if distance.distance(ros.gps, startingPos).m < 7: e
                     lapCounter += 1
                     timeMarker = time.time() #resets the time marker
                     if lapCounter == 10: #change to 10 in the future
                         raise KeyboardInterrupt
    """

    ros.publishCommands(steering, velocity)

    print("Steering: %d, Velocity: %d" % (steering, velocity))
    logging.info("Steering: %d, Velocity: %d", steering, velocity)

    print("Seconds it took: ", time.time() - startTime)
    print("Actual framerate: ", 1 / (time.time() - startTime))

    logging.info("\nTotal seconds: %d", time.time()-startTime)
    logging.info("Framerate: %d", 1 / (time.time() - startTime))
