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

filepath = '/home/eee/catkin_ws/src/SMART_ZED/src/pic_anlg.txt'
img = cv2.imread('/home/eee/Pictures/KAZAMTEST.png')
height, width, depth = img.shape

f = open(filepath, 'w')
for i in range(0, height):
    for j in range(0, width):
        f.write(img[i,j])  
        continue
        f.write('\n')
    break
f.close()
