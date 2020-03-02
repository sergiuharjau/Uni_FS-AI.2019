import cv2
import numpy as np
import time
from bisect import bisect_right
import itertools
import logging

def findGates(red, yellow, depth, firstPass, gateDistance, maxThresh, numberGates):
    """Inputs are red, yellow and depth mask."""

    if firstPass:
        findGates.result = []

    conesDepth = cv2.bitwise_and(depth, depth, mask=red + yellow)
    # depth info just where the cones are
    #print("Len depth", len(depth))
    #print("Len red", len(red))
    #for element in conesDepth:
       #for x in element:
         #print(x, end=", ")
       #print()
    #print("Gate distance", gateDistance)
    #print("Max thresh", maxThresh)
    conesDepth[conesDepth < gateDistance] = np.nan #used to be np.nan
    conesDepth[conesDepth > maxThresh] = np.nan #used to be np.nan
    planeDistance = np.nanmin(conesDepth)
    #print(conesDepth)
    #print("Plane distance: ", planeDistance)

    if np.isnan(planeDistance) or np.isinf(planeDistance):
        print("Stopped looking for gates.")
        return None

    print("Gate distance: ", round(planeDistance,2))

    maxFirstGate = planeDistance + 1

    conesDepth[conesDepth > maxFirstGate] = np.nan #used to be np.nan

    conesDepth[conesDepth > 1] = 1

    markedPixels = (conesDepth.round() * 255).astype(np.uint8)

    firstRed = cv2.bitwise_and(red, red, mask=markedPixels) 
    firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

    if len(firstRed) > 50 or len(firstYellow) > 50:
        findGates.result.append((firstRed, firstYellow))

    if numberGates:
        findGates(red, yellow, depth, False, maxFirstGate+1, maxThresh, numberGates-1)

if __name__ == "__main__":
    pass
