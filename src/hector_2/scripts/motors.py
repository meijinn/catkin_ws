#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped,Twist,Vector3
from time import sleep
import numpy as np

mh = Adafruit_MotorHAT(addr=0x60)
myMotor1 = mh.getMotor(1)
myMotor2 = mh.getMotor(2)

NODE_NAME_STR = 'inobo_dcmotor'
CMD_VEL_STR = 'joy_teleop/cmd_vel'

TREAD = 0.22 #TRED 0.20m
LIMIT = 1.00#0.70
SPEED_MAX = 255
ANGULAR_BOOST = 1.5

def callback(msg):
    rospy.loginfo("%s ' Linear [%f, %f, %f]"%(CMD_VEL_STR, msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("%s ' Angular [%f, %f, %f]"%(CMD_VEL_STR, msg.angular.x, msg.angular.y, msg.angular.z))

    v_r = (-1*msg.angular.z*ANGULAR_BOOST*TREAD/2 + msg.linear.x)*SPEED_MAX*LIMIT
    v_l = (msg.angular.z*ANGULAR_BOOST*TREAD/2 + msg.linear.x)*SPEED_MAX*LIMIT


    if v_r > 0 and v_l > 0:
        rospy.loginfo("forward!!")
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)
        myMotor1.setSpeed(abs(int(v_r)))
        myMotor2.setSpeed(abs(int(v_l)))

    elif v_r > 0 and v_l < 0:
        rospy.loginfo("right!!")
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        myMotor2.run(Adafruit_MotorHAT.FORWARD)
        myMotor1.setSpeed(abs(int(v_r)))
        myMotor2.setSpeed(abs(int(v_l)))

    elif v_r < 0 and v_l > 0:
        rospy.loginfo("left!!")
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)
        myMotor1.setSpeed(abs(int(v_r)))
        myMotor2.setSpeed(abs(int(v_l)))

    elif v_r < 0 and v_l < 0:
        rospy.loginfo("back!!")
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.FORWARD)
        myMotor1.setSpeed(abs(int(v_l)))
        myMotor2.setSpeed(abs(int(v_r)))

    else:
        myMotor1.run(Adafruit_MotorHAT.RELEASE)
        myMotor2.run(Adafruit_MotorHAT.RELEASE)

    #myMotor1.setSpeed(abs(int(v_r)))
    #myMotor2.setSpeed(abs(int(v_l)))

rospy.init_node(NODE_NAME_STR, anonymous=False)
try:
    rospy.Subscriber(CMD_VEL_STR, Twist, callback)
except rospy.ROSInterruptException:
    import traceback
    traceback.print_exc()
rospy.spin()