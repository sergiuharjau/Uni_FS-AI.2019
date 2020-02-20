#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
    print("Subscribed to topic")
    self.cv_image = None

  def callback(self,data):
    print("Got image")
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.imshow("image", self.cv_image)
    #cv2.waitKey(3)

def mainRosNode(args=[]):

  print("Creating ROS node to grab OpenCV images.")
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
    ic = image_converter()
    mainRosNode(sys.argv)