import sys
import time
import threading
from imProc import imProcessing
from capturing import ImageCap
from cmds import issueCommands, calculateReading
from globals import *


def main(visual=False, green=False, record=False, replay=False, loop=True):

    calculateReading.pastCom = 0  # in case we don't see cones straight away

    ic = ImageCap(False, replay)  # ImCapt() #initializes zed object

    startTime = time.time()

    listReadings = []

    try:
        while loop:  # for amount in range(framesToDo):

            image, depth = ic.latest(record)

            t = threading.Thread(target=ImageCap.capture, args=(ic, ))

            original_image = image
            depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
            image_ocv = original_image[startFrom:startFrom + pixelStrip]

            reading = imProcessing(image_ocv, depth_data_ocv, visual, original_image, green)
            t.start()  # works faster for performance reasons

            if ic.exit:  # when we replay tests
                raise KeyboardInterrupt

            if reading:
                print("Camera: ", reading)
                #issueCommands((reading/steeringFactor)*-1, carVelocity, False, visual)
            else:
                print("No camera reading, pastCom: ", calculateReading.pastCom)
                #issueCommands((calculateReading.pastCom/steeringFactor)*-1, carVelocity, False, visual)

            listReadings.append(reading)

            if type(loop) != bool :
                loop -= 1
                if loop == 0:
                    raise KeyboardInterrupt
            #print("Frames left: ", framesToDo-amount)

    except KeyboardInterrupt:
        # issueCommands(0, 0, True, visual) #initiates the exit protocol
        if not replay:
            ic.zed.close()
        if record:
            f1 = open(ic.newMission + "benchmarkCmds.txt", "w")
            for element in listReadings:
                f1.write(str(element) + ",")
            f1.close()
        f2 = open("../test/pastMission.txt", "w")
        for element in listReadings:
            f2.write(str(element) + ",")
        f2.close()

        print("Seconds it took: ", time.time() - startTime)
        print("Total frames: ", len(listReadings))
        print("Actual framerate: ", len(listReadings) / (time.time() - startTime))
        print("\nFINISH")

        quit()


if __name__ == "__main__":

    visual=False; green=False; record=False; replay=False; loop=True

    for argument in sys.argv[1:]:
        exec(argument)

    main(visual, green, record, replay, loop)
