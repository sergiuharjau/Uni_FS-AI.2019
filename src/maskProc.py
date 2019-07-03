import cv2
import numpy as np
import time
from bisect import bisect_right
import itertools


def findGates(red, yellow, depth, firstPass, gateDistance, numberGates):
    """Inputs are red, yellow and depth mask."""

    if firstPass:
        findGates.result = []

    conesDepth = cv2.bitwise_and(depth, depth, mask=red + yellow)
    # depth info just where the cones are

    conesDepth[conesDepth < gateDistance] = np.nan

    planeDistance = np.nanmin(conesDepth)

    if np.isnan(planeDistance):
        return None

    maxFirstGate = planeDistance + 0.3

    conesDepth[conesDepth > maxFirstGate] = np.nan

    conesDepth[conesDepth > 1] = 1

    markedPixels = (conesDepth.round() * 255).astype(np.uint8)

    firstRed = cv2.bitwise_and(red, red, mask=markedPixels)
    firstYellow = cv2.bitwise_and(yellow, yellow, mask=markedPixels)

    findGates.result.append((firstRed, firstYellow))
    print("Found a gate.")
    if numberGates:
        findGates(red, yellow, depth, False, maxFirstGate+1, numberGates-1)
    # re-Add in the future to allow multiple gate processing


def threshold(a, threshmin=None, threshmax=None, newval=0):
    """Returns an array with values bound between threshmin and threshmax """
    a = np.ma.array(a, copy=True)
    mask = np.zeros(a.shape, dtype=bool)
    if threshmin is not None:
        mask |= (a < threshmin).filled(False)

    if threshmax is not None:
        mask |= (a > threshmax).filled(False)

    a[mask] = newval

    return a


def findMin(bigList, k, firstPass=False):
    """Returns the smallest value bigger than k in the array"""
    if 'flattened_list' not in findMin.__dict__ or firstPass:
        findMin.flattened_list = list(itertools.chain(*bigList))
        print(findMin.flattened_list.sort())
        findMin.flattened_list.sort()

    try:
        min_val = findMin.flattened_list[bisect_right(
            findMin.flattened_list, k)]
    except:
        min_val = 0

    return min_val


if __name__ == "__main__":
    pass
