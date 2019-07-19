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

    gps = GPS()

    while gps.getGPS() == (0,0):
        print("Awaiting GPS lock.")
        time.sleep(1)

    startingPos = gps.getGPS()
   
    if not visual:
        issueCommands(0,0)

    timeMarker = time.time()

    ic = ImageCap(False, replay)  # ImCapt() #initializes zed object

    time.sleep(3) # waits 3 seconds 
    
    startTime = time.time()
    listReadings = []

    lapCounter = 0 

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

                steering, velocity = imProcessing(t, image_ocv, depth_data_ocv, visual, original_image, green, cFlip)
            else:
                steering,velocity = 0,0
                cv2.imshow("image", image)
                cv2.waitKey(1)
             # works faster here for performance reasons

            if ic.exit:  # when we replay tests
                raise KeyboardInterrupt
#            input()

            #if steering <= -2:  #Adjusts leftside steering
            #    steering += 2 #disabling it for temporary purposes
            steering = min(19, max(-19, steering))

            if time.time()-timeMarker > 30: #only checks 10s after we've passed the starting point
                if distance.distance(gps.getGPS(), startingPos).m < 5: #5m within the finish line
                    lapCounter += 1 
                    timeMarker = time.time() 
                    if lapCounter == 2: #change to 10 in the future 
                        raise KeyboardInterrupt

                        
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

            if issueCommands.car.checkEBS():
                raise KeyboardInterrupt
                i+=1

            logging.warning("End of frame.\n\n")

            #print("Frames left: ", framesToDo-amount)

    except KeyboardInterrupt:
        if not replay:
            ic.zed.close()
        issueCommands(0,0, False, visual, replay, record, rc)
        time.sleep(5)
        issueCommands(0, 0, True, visual, replay, record, rc) #initiates the exit protocol

        f1 = open("../test/pastMission.txt", "w")
        for element in listReadings:
            f1.write(str(element) + ",")
        f1.close()

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
