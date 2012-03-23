#!/usr/bin/env python
import roslib; roslib.load_manifest('my_odometry')
import rospy

import sys
import cv

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class main_node:
    def __init__(self):
        rospy.init_node('main_node')

        self.cv_window_name = "OpenCV Image"

        cv.NamedWindow(self.cv_window_name, 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",\
        Image, self.callback)

    def callback(self, data):
        try:
          cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
          print e

        cv.ShowImage(self.cv_window_name, cv_image)
        cv.WaitKey(3)

    def printData(self, data):
        try:
          print data
        except Exception, e:
          print e

        cv.ShowImage(self.cv_window_name, cv_image)
        cv.WaitKey(3)

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

if __name__ == '__main__':
      vn = main_node()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting node."
        cv.DestroyAllWindows()
