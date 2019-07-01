import time
import sys
import numpy as np
import cv2
import serial

from globals import width, newComOffset, missedColourOffset

sys.path.insert(0, '../../fspycan/lib/')
#import fspycan_ext

missedRed = 0
missedYellow = 0


def findLineMarkers(red, yellow, i, visual):

    global missedRed
    global missedYellow

    redIndex = np.where(red == 255)
    try:
        redMarker = (redIndex[1][0], redIndex[0][0])
        missedRed = 0
    except:
        missedRed += 1
        if missedRed > 15:
            print("Can't see blue, turning left", (-1*missedRed*missedColourOffset))
            redMarker = (-1 * missedColourOffset * missedRed, 0)
            # very far left red, middle yellow, turns left
        else:
            print("Missed Red: ", missedRed)
            redMarker = False

    yellowIndex = np.where(yellow == 255)
    try:
        yellowMarker = (yellowIndex[1][0], yellowIndex[0][0])
        missedYellow = 0
    except:
        missedYellow += 1
        if missedYellow > 15:
            print("Can't see yellow, turning right", (missedYellow*missedColourOffset))
            yellowMarker = (1280 + missedColourOffset * missedYellow, 0 )
            # middle red, very far right Yellow, turns right
        else:
            print("Missed Yellow: ", missedYellow)
            yellowMarker = False

    if visual:
        for x in range(i + 1):
            cv2.imshow("gate " + str(i), red + yellow)

        for x in range(i + 1, 5):
            cv2.destroyWindow("gate " + str(x))

    return redMarker, yellowMarker


def calculateReading(target):

    newCom = target - int(width / 2)  # offset from center of image

    if newCom != -int(width / 2):  # when we have a correct reading
        final = calculateReading.pastCom + \
            (newCom - calculateReading.pastCom) / newComOffset
        calculateReading.pastCom = final

        return round(final)


def issueCommands(steering, velocity, exit, visual, replay, record, rc):

    if not replay and not record and not visual and not rc:
        if 'car' not in issueCommands.__dict__:  # only runs once
            #issueCommands.car = fspycan_ext.Car("can0")
            #issueCommands.car.init()
            issueCommands.car = 1
            print("Initiating CAN setup.")
            #issueCommands.car.setupCAN()  # function runs until we finish setup
            print("Setup finished gracefully")

        #issueCommands.car.set_steering_velocity(int(steering), int(velocity))
        # we only set the steering here, the loop runs on a different c++ thread

        if exit:  # can exit protocol
            print("Initiating CAN exit.")
            #issueCommands.car.set_steering_velocity(0, 0)
            time.sleep(4)
            #issueCommands.car.exitCAN()  # runs until we exit gracefully
    elif rc == 1:
        if 'ser' not in issueCommands.__dict__:
            issueCommands.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=5)

        steering = min(24, max(-24, steering))
        print(steering)
        commandSteering = "b " + str(1500+int(17.5*steering)) + " \n"
        print(commandSteering)         
        issueCommands.ser.write(commandSteering.encode())

        commandVelocity = "a " + str(1525 + int((velocity-40)/2)) + " \n"
        issueCommands.ser.write(commandVelocity.encode())
