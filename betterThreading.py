import pyzed.sl as sl
import threading
import cv2; import time

class ImageCap:
	__running = False
	def capture(self):
		image_zed = sl.Mat(1280, 720, sl.MAT_TYPE.MAT_TYPE_8U_C4)
		depth_data_zed = sl.Mat(1280, 720, sl.MAT_TYPE.MAT_TYPE_32F_C1)
		
		runtime = sl.RuntimeParameters()
		runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD
		while self.__running:
			if self.frame == None:
				err = self.zed.grab(runtime)
				if err == sl.ERROR_CODE.SUCCESS:
					#print("We're running now!")
					self.zed.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU)
					self.zed.retrieve_measure(depth_data_zed, sl.MEASURE.MEASURE_DEPTH, sl.MEM.MEM_CPU)
					#print("Stopped running")
					self.frame = (image_zed, depth_data_zed)
				else:
					time.sleep(0.001)

	def start(self):
		self.__running = True
		self.__thread = threading.Thread( target=ImageCap.capture, args=(self,) )
		self.__thread.start()

	def stop(self):
		if self.__running:
			self.__running = False
			self.__thread.join()
			self.zed.close()
		
	def __del__(self):
		if self.__running:
			self.stop()

	def __init__(self):
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

		self.frame = None
		self.__running = False
		self.__thread = None

	def latest(self):
		return self.frame
		
	def clear(self):
		self.frame = None

if __name__ == "__main__":
	ic = ImageCap()
	ic.start()
	
	
	while True:
		latest = ic.latest()
		ic.clear()
		
		if latest == None: continue

			
		original_image = latest[0].get_data()
		depth_data_ocv = latest[1].get_data()[270:300]
		image_ocv = original_image[270:300]
		
		reading = imProcessing(image_ocv, depth_data_ocv)
		
		cv2.imshow("image", original_image)
		cv2.imshow("depth", depth_data_ocv)
		cv2.waitKey(10)
		if reading:
			print()
			print(reading)
		
		
	ic.stop()
	


