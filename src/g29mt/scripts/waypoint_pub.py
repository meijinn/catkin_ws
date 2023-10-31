#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from subprocess import *

def callback(cont_msg):
    if cont_msg.linear.y == 1:
          call(['roslaunch','urg_gmapping','run.launch','use_py:=false'])
    if cont_msg.linear.z == 1:
           kill_node('waypoint_pub')

def kill_node(nodename): 
	p2=Popen(['rosnode','list'],stdout=PIPE) 
	p2.wait() 
	nodelist=p2.communicate() 
	nd=nodelist[0] 
	nd=nd.split("\n") 
	for i in range(len(nd)): 
		tmp=nd[i] 
		ind=tmp.find(nodename) 
		if ind==1: 
			call(['rosnode','kill',nd[i]]) 
			break 

def waypoint_pub():
    rospy.init_node('waypoint_pub')
    rospy.Subscriber('controller', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    waypoint_pub()
