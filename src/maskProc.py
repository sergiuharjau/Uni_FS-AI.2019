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

    conesDepth[conesDepth < gateDistance] = np.nan
    conesDepth[conesDepth > maxThresh] = np.nan
    planeDistance = np.nanmin(conesDepth)

    if np.isnan(planeDistance) or np.isinf(planeDistance):
        logging.info("Stopped looking for gates.")
        return None

    logging.info("Gate distance: %fm", round(planeDistance,2))

    maxFirstGate = planeDistance + 1.5

    conesDepth[conesDepth > maxFirstGate] = np.nan

    conesDepth[conesDepth > 1] = 1

    markedPixels = (conesDepth.round() * 255).astype(np.uint8)

    firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
    firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

    findGates.result.append((firstRed, firstYellow))

    if numberGates:
        findGates(red, yellow, depth, False, maxFirstGate+0.1, maxThresh, numberGates-1)
    # re-Add in the future to allow multiple gate processing


if __name__ == "__main__":
    pass
