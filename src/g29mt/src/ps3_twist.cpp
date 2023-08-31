#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel;

void joy_callback(const sensor_msgs::Joy& joy_msg){
    cmd_vel.linear.x = joy_msg.axes[1]*0.1;
    cmd_vel.angular.z = joy_msg.axes[0]*2;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_twist_publisher");
    ros::NodeHandle nh;

    //publish
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //      //subscribe
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(100);
    
    while(ros::ok()){
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}