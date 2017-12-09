#!/usr/bin/env python
import roslib
roslib.load_manifest('drone5')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

#from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
    self.cam_params = rospy.Subscriber("/ardrone/camera_info", CameraInfo, self.parameterCallback)
    self.cv_image = None
    self.params = None
    self.K = None

  def parameterCallback(self,data):
	self.params = data
	self.K = self.params.K  

  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print e

    (rows,cols,channels) = self.cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(self.cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", self.cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
    except CvBridgeError as e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  #print ic.cv_image
  print ic.K
  try:
    rospy.spin()

  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  

if __name__ == '__main__':
    main(sys.argv)
