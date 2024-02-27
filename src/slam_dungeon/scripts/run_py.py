#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import csv
import os
import rospkg
from state.follow_waypoints import FollowPath
from state.make_waypoints import MakeWaypoints

# Note : This code is based on ROS Melodic and python 2.7 

def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    
    rospy.sleep(1.0)
    
    # parameters
    duration = rospy.get_param("~duration", 0.0)
    distance_tolerance = rospy.get_param("~distance_tolerance", 1.0)
    rospack = rospkg.RosPack()
    pkg_name = "follow_waypoints"
    pkg_path = rospack.get_path(pkg_name)
    path_to_waypoints = rospy.get_param("~path_to_waypoints", pkg_path+"/csv/waypoints.csv")
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['OK', 'NG'])
    sm.userdata.config = {'duration':duration, 'distance_tolerance':distance_tolerance}

    with open(path_to_waypoints, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        points = [row for row in reader]
        sm.userdata.points = points
        
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MakeWaypoints', MakeWaypoints(),
                               transitions={'success': 'FollowPath', 'fail':'NG'})
        smach.StateMachine.add('FollowPath', FollowPath(),
                               transitions={'success': 'OK'})
        
    sis = smach_ros.IntrospectionServer('run', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
