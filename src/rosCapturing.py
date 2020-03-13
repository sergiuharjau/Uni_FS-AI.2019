import sys
import rospy
import cv2
import logging
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import  Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix
from autocross import mainProgram

class Image_Converter:

  def __init__(self, visual, cFlip):
    self.visual = visual
    self.cFlip = cFlip
    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber("image_topic", Image, self.imageCallback)
    self.depth_pub = rospy.Subscriber("depth_topic", Image, self.depthCallback)
    self.depth_pub = rospy.Subscriber("gps_topic", NavSatFix, self.gpsCallback)
    self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    print("Subscribed to topic")

    self.cv_image = np.zeros((720,1280,3), np.uint8)
    self.cv_depth = np.zeros((720,1280,1), np.uint8)

  def depthCallback(self, data):
    try:
      intermediateDepth = self.bridge.imgmsg_to_cv2(data)
      self.cv_depth = cv2.resize(intermediateDepth, (1280, 720))
    except CvBridgeError as e:
      print(e)

  def imageCallback(self, data):
    try:
      intermediate = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image = cv2.resize(intermediate, (1280, 720))
    except CvBridgeError as e:
      print(e)
    mainProgram(self.visual, self.cFlip, self)

  def gpsCallback(self, data):
    self.gps = (data.latitude, data.longitude)

  def latestCamera(self):
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

def mainRosNode():

  print("Creating ROS node to grab OpenCV images.")
  rospy.init_node('Image_Converter', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
    visual= False; green= False; cFlip=0

    for argument in sys.argv[1:]:
        exec(argument)

    ros = Image_Converter(visual, green, cFlip)
    mainRosNode()