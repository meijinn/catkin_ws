#!/usr/bin/env python
# -*- coding: utf-8 -*-
# you use this code when you use clicked_point to csv
import rospy
import sys
import csv
from geometry_msgs.msg import PointStamped

def callback(data):
    if data is not None:
        num = data.header.seq
        x = data.point.x
        y = data.point.y
        with open(path_to_waypoints, 'a') as csvfile:
            waypoint = [str(num), str(x), str(y), "0"]
            rospy.loginfo("num : %d, x : %f, y : %f\n", num, x, y)
            filewriter = csv.writer(csvfile, delimiter = ',')
            filewriter.writerow(waypoint)

rospy.init_node('clicked_point')

node_name = rospy.get_name()
path_to_pkg = rospy.get_param("~path_to_pkg")
file_name = rospy.get_param("~file_name")
rospy.logwarn("%s:*-- parameters --*", node_name)
rospy.logwarn("%s:  path_to_pkg=%s", node_name, path_to_pkg)
rospy.logwarn("%s:  file_name=%s", node_name, file_name)
rospy.logwarn("%s:*---------------------*", node_name)
rospy.loginfo("== clicked_point_csv ==")
path_to_waypoints = path_to_pkg + '/graphs/' + file_name + '.csv'
with open(path_to_waypoints, 'w') as csvfile:
    header = ["num" ,"x" , "y", "q"]
    filewriter = csv.writer(csvfile, delimiter = ',')
    filewriter.writerow(header)

rospy.Subscriber("clicked_point", PointStamped, callback)

rospy.spin()