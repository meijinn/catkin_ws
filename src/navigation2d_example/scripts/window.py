#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np

s1 = ''
s2 = ''
first = f'{s1}'
second = f'{s2}'

def callback(cont_msg):
    if cont_msg.linear.x == 0:
        s1 = 'Handover request.'
        s2 = 'Please Operate Manually.'

def monitor():
    rospy.Subscriber('controller', 10, callback)
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    rospy.init_node('monitor_node')
    r = rospy.Rate(100) # 100hz

    cols = 1600
    rows = 500
    img = np.zeros((rows,cols,3),dtype=np.uint8)

    cv2.putText(img, first, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,255,255),5)
    cv2.putText(img, second, (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,255,255),5)
    #img = np.full((rows, cols, 3), 0, dtype=np.uint8)
    #img[0:rows, 0:cols, 1] = 100

    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
    print("monitor is on.")

    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        r.sleep()

if __name__ == '__main__':
    try:
        monitor()
    except rospy.ROSInterruptException: pass