#!/usr/bin/python

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction

rospy.init_node("stop_move_base")

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()
client.cancel_all_goals()