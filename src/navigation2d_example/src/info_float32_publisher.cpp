#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

std_msgs::Float32 vel;
std_msgs::Float32 shift_count;

void callback(const geometry_msgs::Twist& const_msg){
        vel.data = const_msg.linear.x;
        shift_count.data = const_msg.linear.y;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "info_float32_publisher");
    ros::NodeHandle nh;
    
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, callback);

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("vel_monitor",10);
    ros::Publisher shift_pub = nh.advertise<std_msgs::Float32>("shift_monitor",10);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        vel_pub.publish(vel);
        shift_pub.publish(shift_count);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}