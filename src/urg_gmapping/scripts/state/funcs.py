#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from tf.transformations import euler_from_quaternion
import tf

def get_radian(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    radian = math.atan2(dy, dx)
    return radian

def get_localized_pose(listener, time_limit=4.0, target='map', source='base_link'):
    try:
        listener.waitForTransform(target, source, rospy.Time(0), rospy.Duration(time_limit))
        (trans, rot) = listener.lookupTransform(target, source, rospy.Time(0))
        (_, _, yaw) = euler_from_quaternion(rot)
        return (trans[0], trans[1], yaw)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo(e)

def get_distance(x1, y1, x2, y2):
    d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return d