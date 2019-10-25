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
    Captures frames from the camera at 0.4 hz
    Subscribes to
        /raspicam_node/image
    """
    def __init__(self):
        rospy.loginfo("Setting up the node ...")

        rospy.init_node("frames_logger")
        self.bridge = CvBridge()
        self.image = np.zeros((400,220,1), np.uint8)
        #--- Create the subscriber to the /raspicam topic
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        rospy.loginfo("> Subscriber correctly initialized")
    
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        self.image = cv_image

    def process_image(self,t):
        txt='/home/ubuntu/catkin_ws/catkin_ws/src/frames_logger/images/'+str(t)+'.png'
        #txt='../images/'+str(t)+'.png'
        #cv2.imshow(str(t),self.image)
        # cv2.waitKey(0)
        #cv2.destroyAllWindows()
        path=os.path.join(os.path.expanduser('~'),'catkin_ws','src','perception','frames_logger','images',str(t)+'.png')
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
    fl = FramesLogger()
    fl.run()
