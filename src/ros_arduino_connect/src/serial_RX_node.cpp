#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <unistd.h>
#include <string.h>


void arduino_RX_Callback(const std_msgs::Float32& msg){
    printf("km/h:%f    \r",msg.data);
    fflush(stdout);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "serial_RX_node");

    ros::NodeHandle nh;
    ros::Subscriber arduino_subscriber = nh.subscribe("chatter", 1000, arduino_RX_Callback);
    ros::Rate loop_rate(10);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}