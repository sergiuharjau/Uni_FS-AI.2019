import sys
import time
import threading
from imProc import imProcessing
from capturing import ImageCap
from cmds import issueCommands, calculateReading
from globals import pixelStrip, startFrom, steeringFactor, carVelocity
import cv2

def main(visual, green, record, replay, loop):

    calculateReading.pastCom = 0  # in case we don't see cones straight away

    ic = ImageCap(False, replay)  # ImCapt() #initializes zed object

    startTime = time.time()

    reading = None
    listReadings = []

    try:
        while loop:  # for amount in range(framesToDo):

            image, depth = ic.latest(record)

            t = threading.Thread(target=ImageCap.capture, args=(ic, ))

            if not record:
                original_image = image
                depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
                image_ocv = original_image[startFrom:startFrom + pixelStrip]

                reading = imProcessing(image_ocv, depth_data_ocv, visual, original_image, green)
            else:
                cv2.imshow("image", image)
                cv2.waitKey(1)

            t.start()  # works faster here for performance reasons

            if ic.exit:  # when we replay tests
                raise KeyboardInterrupt

            print("Camera value: ", reading)
            issueCommands( (reading or calculateReading.pastCom) /steeringFactor*-1, carVelocity, False, visual, replay, record)
                                    #pastCom if reading=None
            listReadings.append(int((reading or calculateReading.pastCom)/steeringFactor))

            if not isinstance(loop, bool):
                loop -= 1
                if loop == 0:
                    raise KeyboardInterrupt
            #print("Frames left: ", framesToDo-amount)

    except KeyboardInterrupt:
        # issueCommands(0, 0, True, visual) #initiates the exit protocol
        if not replay:
            ic.zed.close()

        f1 = open("../test/pastMission.txt", "w")
        for element in listReadings:
            f1.write(str(element) + ",")
        f1.close()

        print("Seconds it took: ", time.time() - startTime)
        print("Total frames: ", len(listReadings))
        print("Actual framerate: ", len(listReadings) / (time.time() - startTime))
        print("\nFINISH")

        quit()


if __name__ == "__main__":

    visual= False; green= False; record= False; replay= False; loop= True

    for argument in sys.argv[1:]:
        exec(argument)

    main(visual, green, record, replay, loop)
