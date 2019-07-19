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

def main(visual, green, record, replay, loop, rc, cFlip):

    calculateReading.pastCom = 0  # in case we don't see cones straight away

    ic = ImageCap(False, replay)  # ImCapt() #initializes zed object
    startTime = time.time()
    listReadings = []

    gps = GPS()

    while gps.getGPS() == (0,0):
        print("Waiting on GPS lock.")
        time.sleep(1)

    issueCommands(0,0) #makes connection
    time.sleep(3) #waits 3 seconds


    issueCommands(-24, 0) #sweeps left
    time.sleep(2)
    issueCommands(24, 0) #sweep right
    time.sleep(2)
    issueCommands(0,0)

    startingPos = gps.getGPS()
    running = True
    timeCheck = False
    ebs = False

    try:
        i=0
        while loop:  # for amount in range(framesToDo):
            logging.warning("\n*********\nFrame: " + str(i))
            image, depth = ic.latest(record)
            logging.info("Getting latest image and depth.")
            t = threading.Thread(target=ImageCap.capture, args=(ic, ))

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
            
            if running:
                velocity = 250 #15kph
            else:
                velocity = 0

            if distance.distance(gps.getGPS(), startingPos).m > 10:
                running = False
                timeCheck = time.time()
                if ebs:
                    issueCommands.car.deploy_ebs()
                    raise KeyboardInterrupt

            if timeCheck:
                if time.time()-timeCheck > 5: #after 5 seconds
                    startingPos = gps.getGPS() #resets starting position so we go for 10m
                    running = True #resets velocity to 15kph
                    ebs = True #it will deploy ebs once it reaches 10m again

            print("Steering: ", steering)
            print("Velocity: ", velocity)

            logging.info("Steering: %d, Velocity: %d", steering, velocity)
            
            issueCommands(steering, velocity, False, visual, replay, record, rc)

            listReadings.append(steering)

            if not isinstance(loop, bool):
                loop -= 1
                if loop == 0:
                    raise KeyboardInterrupt

            if issueCommands.car.checkEBS():
                raise KeyboardInterrupt
            i+=1   
            t.join()
            i+=1
            logging.warning("End of frame.\n\n")

            #print("Frames left: ", framesToDo-amount)

    except KeyboardInterrupt:
        if not replay:
            ic.zed.close()
        if not ebs:
            issueCommands(0,0, False, visual, replay, record, rc)
            time.sleep(4)
            issueCommands(0, 0, True, visual, replay, record, rc) #initiates the exit protocol

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
