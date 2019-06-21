import pyzed.sl as sl
import time
import cv2
import os
import numpy as np

class ImageCap:

	def capture(self, replay):
		if replay:
			self.replay(replay)
			return

		err = self.zed.grab(self.runtime)
		if err == sl.ERROR_CODE.SUCCESS:
			self.zed.retrieve_image(self.image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU)
			self.zed.retrieve_measure(self.depth_data_zed, sl.MEASURE.MEASURE_DEPTH, sl.MEM.MEM_CPU)
			self.frame = (self.image_zed.get_data(), self.depth_data_zed.get_data())
		else:
			print(err)
			time.sleep(0.01)

	def latest(self, record):
		if record:
			self.record()
		return self.frame

	def __init__(self, record, replay):

		if replay == 0:
			self.zed = sl.Camera()
			
			init = sl.InitParameters()
			init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
			init.camera_fps = 60 # Set max fps at 100

			init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA
			init.coordinate_units = sl.UNIT.UNIT_METER
			
			err = self.zed.open(init)
			if err != sl.ERROR_CODE.SUCCESS :
				print(repr(err))
				self.zed.close()
				exit(1)
				
			self.runtime = sl.RuntimeParameters()
			self.runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD

		self.image_zed = sl.Mat(1280, 720, sl.MAT_TYPE.MAT_TYPE_8U_C4)
		self.depth_data_zed = sl.Mat(1280, 720, sl.MAT_TYPE.MAT_TYPE_32F_C1)
		

		self.frame = (self.image_zed.get_data(), self.depth_data_zed.get_data())
		self.frames = 0

		self.makeFolder = True
		self.i = 1
		self.exit = False	

	def record(self):

		if self.makeFolder:
			for r, d, f in os.walk("Frames/"):
				missions = sorted(d)
				break
			self.newMission = "Frames/mission" + str(int(missions[-1][7:])+1) +"/"
			os.mkdir(self.newMission)
			self.makeFolder = False
		
		np.save(self.newMission + "image" + str(self.frames), self.frame[0], allow_pickle=True)
		np.save(self.newMission + "depth" + str(self.frames), self.frame[1], allow_pickle=True)
		self.frames+=1


	def replay(self, mission):
		try:
			a = np.load("Frames/mission" + str(mission) + "/image" + str(self.i) + ".npy")
			b = np.load("Frames/mission" + str(mission) + "/depth" + str(self.i) + ".npy")
				
			self.frame = (a,b) 
		except FileNotFoundError:
			print("No more files to go")
			self.exit = True

		self.i+=1

if __name__ == "__main__":
	ic = ImageCap(False)
	while True:
		ic.replay()
		cv2.imshow("image", ic.frame[0])
		cv2.imshow("depth", ic.frame[1])
		cv2.waitKey(10)
