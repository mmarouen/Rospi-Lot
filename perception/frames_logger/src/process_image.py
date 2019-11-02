#!/usr/bin/python

import sys
import rospy
import cv2
import time
import numpy as np
import os

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from cv_bridge              import CvBridge, CvBridgeError

class FramesLogger():
    """
    Captures different frames at 0.4 hz and stores to disk
    Subscribes to
        /raspicam_node/image
        /perception/lanes
    """
    def __init__(self):
        rospy.loginfo("Setting up the node ...")

        rospy.init_node("frames_logger")
        self.bridge = CvBridge()
        self.image = np.zeros((400,220,1), np.uint8)
        self.lanes = np.zeros((400,220,1), np.uint8)
        #--- Create the subscriber to the /raspicam topic
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        self.lanes_sub = rospy.Subscriber("/perception/lanes",Image,self.callback2)
        rospy.loginfo("> Subscriber correctly initialized")
    
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        self.image = cv_image

    def callback2(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="mono8")
        except CvBridgeError as e:
            print(e)
        self.lanes = cv_image

    def log_image(self,t):
        path_image=os.path.join(os.path.expanduser('~'),'catkin_ws','src','perception','frames_logger','images',str(t)+'_image.png')
        res=cv2.imwrite(path_image, self.image)
        print("raw image "+str(res))
        path_lanes=os.path.join(os.path.expanduser('~'),'catkin_ws','src','perception','frames_logger','images',str(t)+'_lanes.png')
        res=cv2.imwrite(path_lanes, self.lanes)
        print("lanes image "+str(res))
    
    def run(self):

        #- set the control rate
        rate = rospy.Rate(0.4)
        start=time.time()
        duration = 30#in s
        while (time.time()-start<duration):
            t=int(time.time())
            self.log_image(t)
            rate.sleep()

"""
Execute main function
"""
if __name__ == "__main__":
    fl = FramesLogger()
    fl.run()
