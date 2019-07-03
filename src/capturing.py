import pyzed.sl as sl
import time
import cv2
import os
import numpy as np
from globals import width, height


class ImageCap:

    def capture(self):
        if self.missionNo:
            self.replay(self.missionNo)
            return None

        err = self.zed.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(width), int(height))
            self.zed.retrieve_measure(self.depth_data_zed, sl.MEASURE.MEASURE_DEPTH, sl.MEM.MEM_CPU, int(width), int(height))

            self.frame = (self.image_zed.get_data(), self.depth_data_zed.get_data())
        else:
            print(err)
            time.sleep(0.01)

    def latest(self, record):
        if record:
            self.record()
        return self.frame

    def __init__(self, record, replay):

        self.frames = 0
        self.missionNo = replay

        if not replay:
            self.zed = sl.Camera()

            init = sl.InitParameters()

            init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
            init.coordinate_units = sl.UNIT.UNIT_METER
            init.camera_fps = 30  # Set max fps at 100
            init.depth_minimum_distance = 0.3 #in meters


            init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA


            err = self.zed.open(init)
            if err != sl.ERROR_CODE.SUCCESS:
                print(repr(err))
                self.zed.close()
                exit(1)

            self.runtime = sl.RuntimeParameters()
            self.runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD

            self.image_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
            self.depth_data_zed = sl.Mat(width, height, sl.MAT_TYPE.MAT_TYPE_32F_C1)

            self.frame = (self.image_zed.get_data(), self.depth_data_zed.get_data())
            self.makeFolder = True
            self.exit=False

        else:
            self.exit = False
            self.replay(self.missionNo)


    def record(self):

        if self.makeFolder:
            for r, d, f in os.walk("../test/"):
                missions = sorted(d)
                break
            self.newMission = "../test/mission" + str(int(missions[-1][7:]) + 1) + "/"
            os.mkdir(self.newMission)
            self.makeFolder = False

        np.save(self.newMission + "image" + str(self.frames), self.frame[0], allow_pickle=True)
        np.save(self.newMission + "depth" + str(self.frames), self.frame[1], allow_pickle=True)
        self.frames += 1

    def replay(self, mission):

        try:
            a = np.load("../test/mission" + str(mission) + "/image" + str(self.frames) + ".npy")
            b = np.load("../test//mission" + str(mission) + "/depth" + str(self.frames) + ".npy")

            self.frame = (a, b)
        except FileNotFoundError:
            print("No more files to go")
            self.exit = True

        self.frames += 1


if __name__ == "__main__":
    ic = ImageCap(False)
    while True:
        ic.replay()
        cv2.imshow("image", ic.frame[0])
        cv2.imshow("depth", ic.frame[1])
        cv2.waitKey(10)
