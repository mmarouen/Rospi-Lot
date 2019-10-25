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

class LaneFinder():
    """
    Gets the raw image from the camera and returns a binary image containing lanes information
    Subscribes to
        /raspicam_node/image
    Publishes
        /perception/lanes 
    """
    def __init__(self):
        rospy.loginfo("Setting up the node ...")

        rospy.init_node("lane_finder")
        self.bridge = CvBridge()
        self.image = np.zeros((400,220,1), np.uint8)

        #--- Create the subscriber to the /raspicam topic
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        rospy.loginfo("> Subscriber correctly initialized")

        #--- Create the publisher
        self._ros_pub_lanes = rospy.Publisher("/perception/lanes",Image,queue_size=1)
        rospy.loginfo("> Publisher correctly initialized")
    
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        self.image = cv_image

    def process_image(self,t):
        
        path=os.path.join(os.path.expanduser('~'),'catkin_ws','src','perception','image_processor','images',str(t)+'.png')
        print(path)
        res=cv2.imwrite(path, self.image)

        #res=cv2.imwrite(txt,self.image)
        print("res "+str(res))
    
    def run(self):

        #- set the control rate
        rate = rospy.Rate(0.4)
        start=time.time()
        duration = 10#in s
        while (time.time()-start<duration):
            t=int(time.time())
            self.process_image(t)
            rate.sleep()

"""
Execute main function
"""
if __name__ == "__main__":
    ip = ImageProcessor()
    ip.run()
