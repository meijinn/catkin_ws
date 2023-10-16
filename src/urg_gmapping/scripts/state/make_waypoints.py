#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import tf
from geometry_msgs.msg import Quaternion, PoseStamped

# Note : This code is based on ROS Melodic and python 2.7 

class MakeWaypoints(smach.State):
    def __init__(self, frame_id="map"):
        smach.State.__init__(self, outcomes=['success', 'fail'], input_keys=['points'], output_keys=['waypoints'])
        self.frame_id = frame_id

    def get_waypoints(self, points):
        waypoints = []
        for i in range(0, len(points)):
            wp = PoseStamped()
            wp.header.frame_id = self.frame_id
            wp.header.stamp = rospy.Time.now()
            wp.pose.position.x = float(points[i][1])
            wp.pose.position.y = float(points[i][2])
            wp.pose.position.z = 0
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            wp.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            waypoints.append(wp)
        return waypoints

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)  
        points = userdata.points        
        try:
            waypoints = self.get_waypoints(points)
            userdata.waypoints = waypoints
            return 'success'
        except IndexError:
            rospy.logerr("IndexError")
            return 'fail'
