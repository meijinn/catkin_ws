#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from tf.transformations import euler_from_quaternion
import math

# Note : This code is based on ROS Melodic and python 2.7 

def get_localized_pose(listener, time_limit=10.0, target='map', source='base_link'):
    try:
        listener.waitForTransform(
            target, source, rospy.Time(), rospy.Duration(time_limit))
        (trans, rot) = listener.lookupTransform(
            target, source, rospy.Time(0))
        (_, _, yaw) = euler_from_quaternion(rot)
        return (trans[0], trans[1], yaw)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        raise e

def get_euclidean_distance(x0, y0, x1, y1):
    return math.sqrt(math.pow((x1-x0),2)+math.pow((y1-y0),2))

def get_radian(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    radian = math.atan2(dy, dx)
    return radian