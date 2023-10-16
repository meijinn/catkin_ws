#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import os
import csv
import sys
from blessed import Terminal
from state.funcs import get_localized_pose

def main():
    rospy.init_node('key_click_csv')

    node_name = rospy.get_name()
    path_to_pkg = rospy.get_param("~path_to_pkg")
    file_name = rospy.get_param("~file_name")
    rospy.logwarn("%s:*-- parameters --*", node_name)
    rospy.logwarn("%s:  path_to_pkg=%s", node_name, path_to_pkg)
    rospy.logwarn("%s:  file_name=%s", node_name, file_name)
    rospy.logwarn("%s:*---------------------*", node_name)
    path_to_waypoints = path_to_pkg + '/graphs/' + file_name + '.csv'
    with open(path_to_waypoints, 'w') as csvfile:
        header = ["num" ,"x" , "y", "q"]
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
            (x, y, _) = get_localized_pose(listener)
            waypoint = [str(cnt), str(x), str(y), "0"]
            # rospy.loginfo(waypoint)
            k = t.inkey(timeout=0.001)
            if k.name == 'KEY_ENTER':
                with open(path_to_waypoints, 'a') as csvfile:
                    filewriter = csv.writer(csvfile, delimiter = ',')
                    filewriter.writerow(waypoint)
                rospy.loginfo("num : %d, x : %f, y : %f\n", cnt, x, y)
                cnt += 1
            elif k.name == 'KEY_ESCAPE':
                sys.exit(0)
if __name__ == '__main__':
    main()