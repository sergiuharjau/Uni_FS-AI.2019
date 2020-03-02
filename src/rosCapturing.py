#!/usr/bin/env python3

import sys
import rospy
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from autocross import mainProgram
import logging

class Image_converter:

  def __init__(self, visual, green, record, replay, loop, rc, cFlip):
    self.visual = visual
    self.green = green
    self.record = record
    self.replay = replay
    self.loop = loop
    self.rc = rc
    self.cFlip = cFlip
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic", Image, self.callback)
    self.depth_pub = rospy.Subscriber("depth_topic", Image, self.depthCallback)
    self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    print("Subscribed to topic")
    height = 720 ; width = 1280
    self.cv_image = np.zeros((height,width,3), np.uint8)
    self.cv_depth = np.zeros((height,width,1), np.uint8)

  def depthCallback(self, data):
    try:
      intermediateDepth = self.bridge.imgmsg_to_cv2(data)
      self.cv_depth = cv2.resize(intermediateDepth, (1280, 720))
    except CvBridgeError as e:
      print(e)

  def callback(self, data):
    try:
      intermediate = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image = cv2.resize(intermediate, (1280, 720))
    except CvBridgeError as e:
      print(e)

    mainProgram(self.visual, self.green, self.record, self.replay, 1, self.rc, self.cFlip, self)

  def latest(self):
    return self.cv_image, self.cv_depth

  def pub(self, steering, velocity):
    vel_msg = Twist()
    vel_msg.linear.x = velocity
    vel_msg.angular.z = steering / 15 #to get the right data in cmd_vel
    if vel_msg.angular.z > 0.7:
        vel_msg.angular.z=0.7
    elif vel_msg.angular.z < -0.7:
        vel_msg.angular.z=-0.7
    self.publisher.publish(vel_msg)


def mainRosNode(args=[]):

  print("Creating ROS node to grab OpenCV images.")
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
    visual= False; green= False; record= False; replay= False; loop= True; rc=False; cFlip=0 ;

    for argument in sys.argv[1:]:
        exec(argument)

    ic = Image_converter(visual, green, record, replay, loop, rc, cFlip)
    mainRosNode(sys.argv)
