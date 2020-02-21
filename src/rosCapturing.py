#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

class Image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
    print("Subscribed to topic")
    height = 720 ; width = 1280
    self.cv_image = np.zeros((height,width,3), np.uint8)
    print(self.cv_image)
    self.cv_depth = np.zeros((height,width,1), np.uint8)

  def callback(self,data):
    print("Got image"*25)
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print(type(self.cv_image))
      input()
    except CvBridgeError as e:
      print(e)

  def latest(self):
    return self.cv_image, self.cv_depth

    #cv2.imshow("image", self.cv_image)
    #cv2.waitKey(3)

def mainRosNode(args=[]):

  print("Creating ROS node to grab OpenCV images.")
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
    ic = Image_converter()
    mainRosNode(sys.argv)