#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import os
import csv
import sys
import math
from tf.transformations import euler_from_quaternion
from state.funcs import get_distance, get_localized_pose

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

    listener = tf.TransformListener()
    cnt = 0
    distance = 0
    angle = 0
    rospy.loginfo("== wayponit generator ==")
    path_to_waypoints = path_to_pkg + '/graphs/' + file_name + '.csv'
    with open(path_to_waypoints, 'w') as csvfile:
        header = ["num" ,"x" , "y", "q"]
        filewriter = csv.writer(csvfile, delimiter = ',')
        filewriter.writerow(header)
    (sx, sy, syaw) = get_localized_pose(listener)
    stheta = math.degrees(syaw)
    while not rospy.is_shutdown():
        (x, y, yaw) = get_localized_pose(listener)
        theta = math.degrees(yaw)
        distance = get_distance(sx, sy, x, y)
        angle = abs(theta-stheta)
        # rospy.loginfo("distance: %.2f, distance-dist_th: %.2f", distance, dist_th-distance)
        # rospy.loginfo("angle: %.2f, angle-theta_th: %.2f", angle, abs(angle-theta_th))
        if distance > dist_th or angle > theta_th:
            waypoint = [str(cnt), str(x), str(y), "0"]
            # rospy.loginfo(waypoint)
            with open(path_to_waypoints, 'a') as csvfile:
                filewriter = csv.writer(csvfile, delimiter = ',')
                filewriter.writerow(waypoint)
            rospy.loginfo("num : %d, x : %f, y : %f\n", cnt, x, y)
            cnt += 1
            (sx, sy, syaw) = get_localized_pose(listener)
            stheta = math.degrees(syaw)
            distance = 0
            angle = 0
                
if __name__ == '__main__':
    main()