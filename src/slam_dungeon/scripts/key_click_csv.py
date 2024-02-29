#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
import tf
import os
import csv
import sys
from blessed import Terminal
from state.funcs import get_localized_pose

bridge = CvBridge()

def callback(msg):
    global temp
    temp = msg.data

def rgbcallback(msg):
    global rgb
    rgb = msg.data


def Imgcallback(image_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        # cv2.imshow('ROS Image Subscriber', cv_image)
        cv2.waitKey(10)
    except CvBridgeError as error:
        print(error)

def main():
    rospy.init_node('lidar_temp_img_csv')

    node_name = rospy.get_name()
    path_to_pkg = rospy.get_param("~path_to_pkg")
    file_name = rospy.get_param("~file_name")
    rospy.logwarn("%s:*-- parameters --*", node_name)
    rospy.logwarn("%s:  path_to_pkg=%s", node_name, path_to_pkg)
    rospy.logwarn("%s:  file_name=%s", node_name, file_name)
    rospy.logwarn("%s:*---------------------*", node_name)

    print("Subscribe images from topic /image_raw ...")
    rospy.Subscriber("image_raw", Image, Imgcallback)

    path_to_waypoints = path_to_pkg + '/graphs/' + file_name + '.csv'
    with open(path_to_waypoints, 'w') as csvfile:
        header = ["num", "x", "y", "temp", "RGB"]
        filewriter = csv.writer(csvfile, delimiter = ',')
        filewriter.writerow(header)
    listener = tf.TransformListener()
    cnt = 0
    t = Terminal()
    rospy.loginfo("== key_click_csv ==")
    rospy.loginfo("Press the 'Enter' to save localized pose")
    rospy.loginfo("Press the 'esc' to end this file\n")
    with t.cbreak():
        while not rospy.is_shutdown():
            rospy.Subscriber('temp', Float32, callback)
            rospy.Subscriber('RGB', String, rgbcallback)
            (x, y, _) = get_localized_pose(listener)
            waypoint = [str(cnt), str(x), str(y), str(temp), rgb]
            # rospy.loginfo(waypoint)
            k = t.inkey(timeout=0.001)
            if k.name == 'KEY_ENTER':
                with open(path_to_waypoints, 'a') as csvfile:
                    filewriter = csv.writer(csvfile, delimiter = ',')
                    filewriter.writerow(waypoint)
                rospy.loginfo("num : %d, x : %f, y : %f, temp : %f, RGB: %s\n", cnt, x, y, temp, rgb)
                cnt += 1
            elif k.name == 'KEY_ESCAPE':
                sys.exit(0)
if __name__ == '__main__':
    main()