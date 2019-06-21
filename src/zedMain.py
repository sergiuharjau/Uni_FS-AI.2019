import sys
import time
import threading
from imProc import imProcessing
from capturing import ImageCap
from cmds import issueCommands, calculateReading
from globals import *


def main(visual=False, green=False, record=False, replay=False):

    calculateReading.pastCom = 0  # in case we don't see cones straight away
    if len(sys.argv) > 1:
        visual = sys.argv[1]
        if len(sys.argv) > 2:
            green = sys.argv[2]

    ic = ImageCap(False, replay)  # ImCapt() #initializes zed object

    startTime = time.time()

    listReadings = []

    framesToDo = 200

    try:
        while True:  # for amount in range(framesToDo):

            image, depth = ic.latest(record)

            t = threading.Thread(target=ImageCap.capture, args=(ic, ))

            original_image = image
            depth_data_ocv = depth[startFrom:startFrom + pixelStrip]
            image_ocv = original_image[startFrom:startFrom + pixelStrip]

            reading = imProcessing(image_ocv, depth_data_ocv, visual, original_image, green)
            t.start()  # works faster for performance reasons

            if ic.exit:  # when we replay tests
                raise(KeyboardInterrupt)

            if reading:
                print("Camera: ", reading)
                #issueCommands((reading/steeringFactor)*-1, carVelocity, False, visual)
            else:
                print("No camera reading, pastCom: ", calculateReading.pastCom)
                #issueCommands((calculateReading.pastCom/steeringFactor)*-1, carVelocity, False, visual)

            listReadings.append(reading)
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
        quit()

    ic.zed.close()
    print("Seconds it took: ", time.time() - startTime)
    print("Actual framerate: ", framesToDo / (time.time() - startTime))
    print("\nFINISH")


if __name__ == "__main__":
    main(record=False, replay=5)
