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
        #attributes
        self.bridge = CvBridge()
        self.image = np.zeros((400,220,1), np.uint8)

        ### parameters
        self._GRAYSCALE_THRESHOLDS = [(240, 255)] # White Line.
        self._GRAYSCALE_HIGH_LIGHT_THRESHOLDS = [(250, 255)]
        self._AREA_THRESHOLD = 0 # Raise to filter out false detections.
        self._PIXELS_THRESHOLD = 40 # Raise to filter out false detections.

        self._Yellow=False
        self._MAG_THRESHOLD = 3 #magnitude of line in hough transform
        self._THD_WHITE=(240,255) #threshold for both yellow and white
        self._THD_YELLOW=(180,255)
        self._THD=self._THD_YELLOW if self._Yellow else self._THD_WHITE


        #--- Create the subscriber to the /raspicam topic
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        rospy.loginfo("> Subscriber correctly initialized")

        #--- Create the publisher
        self._ros_pub_lanes = rospy.Publisher("/perception/lanes",Image,queue_size=1)
        rospy.loginfo("> Publisher correctly initialized")
    
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="mono8")
        except CvBridgeError as e:
            print(e)
        self.image = cv_image

    def process_image(self):
        #self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.image= cv2.equalizeHist(self.image)
        ret,self.image = cv2.threshold(self.image,127,255,cv2.THRESH_BINARY)
        kernel = np.ones((3,3),np.uint8)
        self.image = cv2.erode(self.image,kernel,iterations = 1)
        self._ros_pub_lanes.publish(self.bridge.cv2_to_imgmsg(self.image))
    
    def run(self):

        #- set the control rate
        rate = rospy.Rate(0.4)
        start=time.time()
        duration = 15#in s
        while (time.time()-start<duration):
            self.process_image()
            rate.sleep()

"""
Execute main function
"""
if __name__ == "__main__":
    ip = LaneFinder()
    ip.run()
