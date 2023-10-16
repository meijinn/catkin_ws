#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import sys
import csv
import math
import tf
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Quaternion
from state.funcs import get_radian


class MakerDataType(object):
    ARROW = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    LINE_STRIP = 4
    LINE_LIST = 5
    CUBE_LIST = 6
    SPHERE_LIST = 7
    POINTS = 8

def main():
    rospy.init_node("publish_maker")

    # parameters
    node_name = rospy.get_name()
    distance_tolerance = rospy.get_param("~distance_tolerance")
    path_to_pkg = rospy.get_param("~path_to_pkg")
    file_name = rospy.get_param("~file_name")
    rospy.logwarn("%s:*-- parameters --*", node_name)
    rospy.logwarn("%s:  distance_tolerance=%s", node_name, distance_tolerance)
    rospy.logwarn("%s:  path_to_pkg=%s", node_name, path_to_pkg)
    rospy.logwarn("%s:  file_name=%s", node_name, file_name)
    rospy.logwarn("%s:*---------------------*", node_name)

    pub = rospy.Publisher("waypoints", Marker, queue_size=10)

    rate = rospy.Rate(25)
    rospy.loginfo("== publish_maker ==")
    path_to_waypoints = path_to_pkg + '/graphs/' + file_name + '.csv'
    while not rospy.is_shutdown():
        with open(path_to_waypoints, 'r') as f:
            counter = 0
            reader = csv.reader(f)
            header = next(reader)
            waypoints = [row for row in reader]
            for i in range(len(waypoints)):
                x1 = float(waypoints[i][1])
                y1 = float(waypoints[i][2])
                if i != len(waypoints) - 1:
                    x2 = float(waypoints[i+1][1])
                    y2 = float(waypoints[i+1][2])
                    theta = float(math.degrees(get_radian(x1, y1, x2, y2)))
                else:
                    theta = float(waypoints[i][3])
                # Mark arrow
                marker_data = Marker()
                marker_data.header.frame_id = "map"
                marker_data.header.stamp = rospy.Time.now()

                marker_data.ns = "basic_shapes"
                marker_data.id = counter

                marker_data.action = Marker.ADD

                marker_data.pose.position.x = x1
                marker_data.pose.position.y = y1
                marker_data.pose.position.z = 0

                q = tf.transformations.quaternion_from_euler(
                    0, 0, math.radians(theta))
                marker_data.pose.orientation = Quaternion(
                    q[0], q[1], q[2], q[3])

                marker_data.color.r = 0.0
                marker_data.color.g = 0.0
                marker_data.color.b = 1.0
                marker_data.color.a = 0.5
                marker_data.scale.x = 0.6
                marker_data.scale.y = 0.05
                marker_data.scale.z = 0.1

                marker_data.lifetime = rospy.Duration()

                # marker_data.type = 0
                marker_data.type = MakerDataType.ARROW
                pub.publish(marker_data)
                counter += 1

                # Mark num
                marker_data = Marker()
                marker_data.header.frame_id = "map"
                marker_data.header.stamp = rospy.Time.now()

                marker_data.ns = "basic_shapes"
                marker_data.id = counter

                marker_data.action = Marker.ADD

                marker_data.pose.position.x = x1
                marker_data.pose.position.y = y1
                marker_data.pose.position.z = 0.25  # x or y or z div 2

                q = tf.transformations.quaternion_from_euler(
                    0, 0, math.radians(theta))
                marker_data.pose.orientation = Quaternion(
                    q[0], q[1], q[2], q[3])

                marker_data.color.r = 1.0
                marker_data.color.g = 1.0
                marker_data.color.b = 1.0
                marker_data.color.a = 1.0
                marker_data.scale.x = 0.5
                marker_data.scale.y = 0.5
                marker_data.scale.z = 0.5

                marker_data.lifetime = rospy.Duration()

                marker_data.type = Marker.TEXT_VIEW_FACING
                marker_data.text = str(waypoints[i][0])

                pub.publish(marker_data)
                counter += 1

                # Mark SPHERE
                marker_data = Marker()
                marker_data.header.frame_id = "map"
                marker_data.header.stamp = rospy.Time.now()

                marker_data.ns = "basic_shapes"
                marker_data.id = counter

                marker_data.action = Marker.ADD

                marker_data.pose.position.x = x1
                marker_data.pose.position.y = y1
                marker_data.pose.position.z = 0

                q = tf.transformations.quaternion_from_euler(
                    0, 0, math.radians(theta))
                marker_data.pose.orientation = Quaternion(
                    q[0], q[1], q[2], q[3])

                marker_data.color.r = 0.0
                marker_data.color.g = 0.0
                marker_data.color.b = 1.0
                marker_data.color.a = 0.2
                # if you change distance_tolerance you have to change the rate
                marker_data.scale.x = distance_tolerance * 2
                marker_data.scale.y = distance_tolerance * 2
                marker_data.scale.z = 0.01

                marker_data.lifetime = rospy.Duration()

                # marker_data.type = 3
                marker_data.type = MakerDataType.SPHERE
                pub.publish(marker_data)
                counter += 1
            rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
