#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

from svcam.msg import imagepoints

class image_converter:  
    def __init__(self):
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_converter/rgb_image", Image, self.callback)

    def callback(self, data):
        

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8:CV_8UC3")
        bwimg = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("cv_image", bwimg)       

        cv2.waitKey(3)


if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
   
