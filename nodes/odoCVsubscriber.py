#!/usr/bin/env python
import roslib; roslib.load_manifest('my_odometry')
import rospy

import sys
import cv

from std_msgs.msg import String
from sensor_msgs.msg import Image as ImageMsg # Otherwise there's a collision with TK.image or something similar
from cv_bridge import CvBridge, CvBridgeError

#January 17th => Temporary GUI for testing
import Tkinter
from Tkinter import *

class main_node:
    def __init__(self):
        rospy.init_node('openCVodometry')

        self.cv_window_name = "OpenCV Odometry"

        cv.NamedWindow(self.cv_window_name, 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",\
        ImageMsg, self.callback)

        root = Tk()
        app = TKWindow(master=root)
        try:
            app.mainloop()
            root.destroy()
        except KeyboardInterrupt:
            print "Shutting node."
            cv.DestroyAllWindows()

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



class TKWindow(Frame):
    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.quit

        self.QUIT.pack({"side": "left"})

        self.command1 = Button(self)
        self.command1["text"] = "show_accel",
#        self.command1["command"] = lambda: method2("show_accel")
        self.command1.pack
        self.command1.pack({"side": "bottom"})
        self.command1.pack({"side": "left"})

        self.command2 = Button(self)
        self.command2["text"] = "show_status",
#        self.command2["command"] = method1
        self.command2.pack({"side": "bottom"})
        self.command2.pack({"side": "left"})
        
        
        self.can = Canvas(self.master, width=500, height=250)
        self.can.grid(row=2, column=1)
        self.can.create_line(0,0,500,200)

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.grid()
        self.createWidgets()

if __name__ == '__main__':
    vn = main_node()
