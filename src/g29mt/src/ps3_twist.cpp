#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel;

double forward, back = 0.0;

void joy_callback(const sensor_msgs::Joy& joy_msg){
    forward = (-joy_msg.axes[5]+1)/2;
    back = (joy_msg.axes[2]-1)/2;
    cmd_vel.linear.x = (forward+back)*0.1;
    if(cmd_vel.linear.x < 0){
        cmd_vel.angular.z *= -1;
    }
    cmd_vel.angular.z = 0.5*joy_msg.axes[0];
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