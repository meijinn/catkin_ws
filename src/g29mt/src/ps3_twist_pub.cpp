#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist controller;

double forward, back = 0.0;

void joy_callback(const sensor_msgs::Joy& joy_msg){
    forward = (-joy_msg.axes[5]+1)/2;
    back = (joy_msg.axes[2]-1)/2;
    controller.linear.x = (forward+back)*0.1;
    if(controller.linear.x < 0){
        controller.angular.z = -0.5*joy_msg.axes[0];
    }
    else{
        controller.angular.z = 0.5*joy_msg.axes[0];
    }
    controller.linear.z = (int)(joy_msg.buttons[1]);
    controller.linear.y = (int)(joy_msg.buttons[2]);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_twist_publisher");
    ros::NodeHandle nh;

    //publish
    ros::Publisher controller_pub = nh.advertise<geometry_msgs::Twist>("controller", 10);
    //      //subscribe
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(100);
    
    while(ros::ok()){
        controller_pub.publish(controller);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}