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
import math
from tf.transformations import euler_from_quaternion
from state.funcs import get_distance, get_localized_pose

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
        cv2.waitKey(10)
    except CvBridgeError as error:
        print(error)

def main():
    rospy.init_node('threshold_csv')

    node_name = rospy.get_name()
    path_to_pkg = rospy.get_param("~path_to_pkg")
    dist_th = rospy.get_param("~dist_th") # threshold of distance for adding new waypoint
    theta_th = rospy.get_param("~theta_th") # threshold of yaw angle[rad] for adding new waypoint
    file_name = rospy.get_param("~file_name")
    rospy.logwarn("%s:*-- parameters --*", node_name)
    rospy.logwarn("%s:  path_to_pkg=%s", node_name, path_to_pkg)
    rospy.logwarn("%s:  dist_th=%s", node_name, dist_th)
    rospy.logwarn("%s:  theta_th=%s", node_name, theta_th)
    rospy.logwarn("%s:  file_name=%s", node_name, file_name)
    rospy.logwarn("%s:*---------------------*", node_name)

    print("Subscribe images from topic /image_raw ...")
    rospy.Subscriber("image_raw", Image, Imgcallback)

    listener = tf.TransformListener()
    cnt = 0
    distance = 0
    angle = 0
    rospy.loginfo("== wayponit generator ==")
    path_to_waypoints = path_to_pkg + '/graphs/' + file_name + '.csv'
    with open(path_to_waypoints, 'w') as csvfile:
        header = ["num" ,"x" , "y", "temp", "RGB"]
        filewriter = csv.writer(csvfile, delimiter = ',')
        filewriter.writerow(header)
    (sx, sy, syaw) = get_localized_pose(listener)
    stheta = math.degrees(syaw)
    while not rospy.is_shutdown():
        (x, y, yaw) = get_localized_pose(listener)
        theta = math.degrees(yaw)
        distance = get_distance(sx, sy, x, y)
        angle = abs(theta-stheta)
        rospy.Subscriber('temp', Float32, callback)
        rospy.Subscriber('RGB', String, rgbcallback)

        if distance > dist_th or angle > theta_th:
            waypoint = [str(cnt), str(x), str(y), str(temp), rgb]
            # rospy.loginfo(waypoint)
            with open(path_to_waypoints, 'a') as csvfile:
                filewriter = csv.writer(csvfile, delimiter = ',')
                filewriter.writerow(waypoint)
            rospy.loginfo("num : %d, x : %f, y : %f, temp : %f, RGB : %s\n", cnt, x, y, temp, rgb)
            cnt += 1
            (sx, sy, syaw) = get_localized_pose(listener)
            stheta = math.degrees(syaw)
            distance = 0
            angle = 0
                
if __name__ == '__main__':
    main()