#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayMenu.h>
#include <string>
#include <sensor_msgs/Joy.h>

double up;
double down;
jsk_rviz_plugins::OverlayMenu menu;

void joy_callback(const sensor_msgs::Joy& joy_msg){
    up = joy_msg.buttons[13];
    down = joy_msg.buttons[14];
    menu.action = jsk_rviz_plugins::OverlayMenu::ACTION_SELECT;
    if(up)
        menu.current_index = 0;
    if(down)
        menu.current_index = 1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "info_overlay_menu_publisher");
    ros::NodeHandle nh;

    // publisher
    ros::Publisher menu_pub = nh.advertise<jsk_rviz_plugins::OverlayMenu>("menu", 10);

    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(100);

    while(ros::ok()){
        menu.menus.resize(2);
        menu.menus[0] = "Auto";
        menu.menus[1] = "Manual";
        menu.title = "Mode";
        menu_pub.publish(menu);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}