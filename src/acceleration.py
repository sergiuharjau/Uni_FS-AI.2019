import sys
import time
import threading
from imProc import imProcessing
from capturing import ImageCap
from cmds import calculateReading, issueCommands
from globals import pixelStrip, startFrom, logInitial
import cv2
import logging
from gps import GPS
from geopy import distance

def main(visual, green, record, replay, loop, rc, cFlip):

    calculateReading.pastCom = 0  # in case we don't see cones straight away

    ic = ImageCap(False, replay)  # ImCapt() #initializes zed object
    startTime = time.time()
    listReadings = []

    gps = GPS()

    while gps.getGPS(force=True) == (0,0):
        print("Waiting on GPS lock.")
        time.sleep(1)
    issueCommands(0,0) #makes connection
    time.sleep(5)
    startingPos = gps.getGPS(force=True)

    startPulse = int((issueCommands.get_left_pulse() + issueCommands.get_right_pulse())/2)

    try:
        i=0
        while loop:  # for amount in range(framesToDo):
            logging.warning("\n*********\nFrame: " + str(i))
            image, depth = ic.latest(record)
            logging.info("Getting latest image and depth.")
            t = threading.Thread(target=ImageCap.capture, args=(ic, ))

            currentPulse = int((issueCommands.get_left_pulse() + issueCommands.get_right_pulse())/2)

            if not record:
                original_image = image
                depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
                image_ocv = original_image[startFrom:startFrom + pixelStrip]

                steering, velocity = imProcessing(t, image_ocv, depth_data_ocv, visual, original_image, green, cFlip, gates=0)
            else:                                                                                           #looks for one gate
                steering,velocity = 0,0
                cv2.imshow("image", image)
                cv2.waitKey(1)
             # works faster here for performance reasons

            if ic.exit:  # when we replay tests
                raise KeyboardInterrupt

            steering = min(19, max(-19, steering))
            
            velocity = 250

            if distance.distance(gps.getGPS(timeBound=3), startingPos).m > 70: #change to 75 later
                raise KeyboardInterrupt

            if currentPulse - startPulse > 822: #track / (wheelDiam * pi) * pulses 
            	print("Reached end.")
            	#raise KeyboardInterrupt

            print("Steering: ", steering)
            print("Velocity: ", velocity)

            logging.info("Steering: %d, Velocity: %d", steering, velocity)
            
            issueCommands(steering, velocity, False, visual, replay, record, rc)

            listReadings.append(steering)

            if not isinstance(loop, bool):
                loop -= 1
                if loop == 0:
                    raise KeyboardInterrupt
            t.join()
            i+=1
            logging.warning("End of frame.\n\n")

            #print("Frames left: ", framesToDo-amount)

    except KeyboardInterrupt:
        if not replay:
            ic.zed.close()
        issueCommands(2,0) #this is our wheels centered properly
        print("We've set velocity to 0. Waiting to stop now.")
        print("Doing -2 on the can for 7s")
        time.sleep(7) #increase in future if needed
        issueCommands(1,0)
        print("Doing -1 on the can for 10s")
        time.sleep(10)
        issueCommands(2, 0, True, visual, replay, record, rc) #initiates the exit protocol

        print("Seconds it took: ", time.time() - startTime)
        print("Total frames: ", len(listReadings))
        print("Actual framerate: ", len(listReadings) / (time.time() - startTime))
        print("\nFINISH")

        logging.info("\nTotal seconds: %d", time.time()-startTime)
        logging.info("Framerate: %d", len(listReadings) / (time.time() - startTime))
        logging.warning("Mission end.\n\n")
        quit()


if __name__ == "__main__":

    visual= False; green= False; record= False; replay= False; loop= True; rc=False; cFlip=0 ;

    for argument in sys.argv[1:]:
        exec(argument)

    logging.basicConfig(filename='loggingMain.log', format='%(asctime)s %(message)s', level=logging.INFO)
    logging.warning("\nMission start!\n")

    logInitial()

    main(visual, green, record, replay, loop, rc, cFlip)
