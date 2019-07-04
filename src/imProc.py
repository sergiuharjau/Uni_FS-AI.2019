import cv2
from maskProc import findGates
from cmds import findLineMarkers, calculateReading
from colour import findColour
from globals import carVelocity, startFrom, pixelStrip


def imProcessing(image_ocv, depth_data_ocv, visual=False, original_image=None, green=False):

    maskRed, maskYellow, stop = findColour(image_ocv, green)

    if stop:
        print("Attention, pedestrian!")
        raise KeyboardInterrupt

    findGates(maskRed, maskYellow, depth_data_ocv, True, 0.3, 3)
    # finds the masks for the first red/yellow cones
    gateDict = {}
    for i, gate in enumerate(findGates.result):
        fRed = gate[0]
        fYellow = gate[1]
        redLine, yellowLine = findLineMarkers(fRed, fYellow, i, visual)
        if redLine and yellowLine:  # only on correct readings
            target = (int((yellowLine[0] + redLine[0]) / 2),int((yellowLine[1] + redLine[1]) / 2))
            gateDict[i] = [target, redLine, yellowLine]

    steering, velocity =  calculateReading(gateDict)

    if visual:
        colours = [(0,255,0), (255,0,0), (0,0,0), (0,0,255)]
        for key in gateDict:
            cv2.line(original_image[startFrom:startFrom + pixelStrip], gateDict[key][1], gateDict[key][2], colours[key], 10)

        cv2.imshow('colour data', cv2.bitwise_and(image_ocv, image_ocv, mask=maskRed + maskYellow))
        cv2.imshow("image", original_image)
        cv2.imshow("cropped", image_ocv)
        cv2.waitKey(10)

    return steering, carVelocity
