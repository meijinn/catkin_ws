#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from funcs import get_localized_pose, get_euclidean_distance

# Note : This code is based on ROS Melodic and python 2.7 

class FollowPath(smach.State):
    def __init__(self, frame_id="map", odom_frame_id="base_link", base_frame_id="scanmatcher_frame"):
        smach.State.__init__(self, outcomes=['success'], input_keys=['waypoints', 'config'])
        self.frame_id = frame_id
        self.odom_frame_id = odom_frame_id
        self.base_frame_id = base_frame_id

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        waypoints = userdata.waypoints
        duration = userdata.config['duration']
        distance_tolerance = userdata.config['distance_tolerance']
        # Get a move_base action client
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not ac.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Waiting for the move_base action server to come up")
        rospy.loginfo("The server comes up")
        listener = tf.TransformListener()
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' % (waypoint.pose.position.x, waypoint.pose.position.y))
            (x, y, _) = get_localized_pose(listener)
            rospy.loginfo("Global Robot Position (%.2f, %.2f)" % (x, y))
            wx = waypoint.pose.position.x
            wy = waypoint.pose.position.y
            distance = get_euclidean_distance(x, y, wx, wy)
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            ac.send_goal(goal)       
            if not distance_tolerance > 0.0:
                ac.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % duration)
                rospy.sleep(duration)
            while distance > distance_tolerance:
                (x, y, _) = get_localized_pose(listener)
                distance = get_euclidean_distance(x, y, wx, wy)
                rospy.sleep(0.1)
            rospy.loginfo('##### REACHED DISTANCE TOLERANCE #####')
        return 'success'

