#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include <sensor_msgs/Joy.h>

int last_state, state = 0;
bool sw = false;

jsk_rviz_plugins::OverlayText text;
std_msgs::ColorRGBA color1, color2;

void joy_callback(const sensor_msgs::Joy& joy_msg){
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.font = "Ubuntu";
    text.text = "";
    state = joy_msg.buttons[1];
    if(state != last_state){
        if(state == 1){
            sw = !sw;
        }
    }
    last_state = state;
    if(sw == true)
        text.text = "Handover Request. \n Please Operate Manually.";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_overlay_text_publisher");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1);

  // subscriber
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    text.line_width = 1;
    text.text_size = 20;
    text.width = 359;
    text.height = 70;
    text.left = 10;
    text.top = 10;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 0.4;
    text.bg_color = color1;

    color2.r = 25.0 / 255;
    color2.g = 255.0 / 255;
    color2.b = 240.0 / 255;
    color2.a = 0.8;
    text.fg_color = color2;
    text_pub.publish(text);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
