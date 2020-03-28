import sys
import rospy
import cv2
import logging
import numpy as np
from webots_ros.srv  import set_float
from sensor_msgs.msg import  Image, NavSatFix
from cv_bridge import CvBridge, CvBridgeError
from autocross import mainProgram
from sensor_msgs.msg import LaserScan

class Image_Converter:

  def __init__(self, visual, cFlip, autonomous):
    self.visual = visual
    self.cFlip = cFlip
    self.bridge = CvBridge()
    rospy.init_node('Image_Converter', anonymous=True)

    self.image_sub = rospy.Subscriber("/fsai/zedcam_left_camera/image", Image, self.imageCallback)
    self.depth_sub = rospy.Subscriber("/fsai/zedcam_left_depth/range_image", Image, self.depthCallback)
    self.gps_sub = rospy.Subscriber("gps_topic", NavSatFix, self.gpsCallback)
    self.set_steering = rospy.ServiceProxy('/fsai/automobile/set_steering_angle', set_float)
    self.set_velocity = rospy.ServiceProxy('/fsai/automobile/set_cruising_speed', set_float)
    self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    print("Subscribed to topic")

    self.cv_image = np.zeros((720,1280,3), np.uint8)
    self.cv_depth = np.zeros((720,1280,1), np.uint8)

  def depthCallback(self, data):
    try:
      intermediateDepth = self.bridge.imgmsg_to_cv2(data)
      self.cv_depth = cv2.resize(intermediateDepth, (1280, 720))
    except CvBridgeError as e:
      print(e)

    current_time = rospy.Time.now()
    laser_frequency = 40
    slice_at = 400
    num_readings = len(self.cv_depth[slice_at])

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'base_link'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14 / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 9.0

    scan.ranges = []
    scan.intensities = []
    for i in reversed(range(0, num_readings)):
        if self.cv_depth[slice_at][i] >= scan.range_max:
            scan.ranges.append(-1)
        else:
            scan.ranges.append(self.cv_depth[slice_at][i])  # take the values at px 400 horizontally
    #print("Depth", self.cv_depth[slice_at])
    #print("Scan", scan)
    self.scan_pub.publish(scan)

  def imageCallback(self, data):
    try:
      intermediate = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.cv_image = cv2.resize(intermediate, (1280, 720))
    except CvBridgeError as e:
      print(e)
    if autonomous:
        mainProgram(self.visual, self.cFlip, self)

  def gpsCallback(self, data):
    self.gps = (data.latitude, data.longitude)

  def latestCamera(self):
    return self.cv_image, self.cv_depth

  def publishCommands(self, steering, velocity):
    steering = max(min(0.7, steering/20), -0.7)
    self.set_velocity(velocity)
    self.set_steering(steering)

def mainRosNode():
  print("Creating ROS node to grab OpenCV images.")
  #rospy.init_node('Image_Converter', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
    visual= False; cFlip=0; autonomous=1;

    for argument in sys.argv[1:]:
        exec(argument)

    ros = Image_Converter(visual, cFlip, autonomous)
    mainRosNode()
