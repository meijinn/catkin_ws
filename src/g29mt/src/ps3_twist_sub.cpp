#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel;

void cont_callback(const & cont_msg){
    cmd_vel.linear.x = cont_msg.linear.x;
    cmd_vel.angular.z = cont_msg.angular.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "vel_pub_node");
    ros::NodeHandle nh;

    //publish
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //      //subscribe
    ros::Subscriber cmd_sub = nh.subscribe("controller", 10, cont_callback);

    ros::Rate loop_rate(100);
    
    while(ros::ok()){
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}