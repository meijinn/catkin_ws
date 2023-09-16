import rospy
from geometry_msgs.msg import Twist

cmd_vel = Twist()

def cont_callback(cont_msg):
    cmd_vel.linear.x = cont_msg.linear.x
    cmd_vel.angular.z = cont_msg.angular.z

if __name__ == '__main__':
    rospy.init_node('vel_pub_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('controller',Twist, cont_callback)
    pub.publish(cmd_vel)
    rospy.spin()