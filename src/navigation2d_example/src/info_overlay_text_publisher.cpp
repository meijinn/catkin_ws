#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


int last_state, state = 0;
bool sw = false;

jsk_rviz_plugins::OverlayText text;
std_msgs::ColorRGBA color1, color2;

// void joy_callback(const sensor_msgs::Joy& joy_msg){
//     text.action = jsk_rviz_plugins::OverlayText::ADD;
//     text.font = "Ubuntu";
//     text.text = "";
//     state = joy_msg.buttons[1];
//     if(state != last_state){
//         if(state == 1){
//             sw = !sw;
//         }
//     }
//     last_state = state;
//     if(sw == true)
//         text.text = "Handover Request. \n Please Operate Manually.";
// }

void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status){
    int status_id = 0;
    //uint8 PENDING       = 0
    //uint8 ACTIVE        = 1
    //uint8 PREEMPTED     = 2
    //uint8 SUCCEEDED     = 3
    //uint8 ABORTED       = 4
    //uint8 REJECTED      = 5
    //uint8 PREEMPTING    = 6
    //uint8 RECALLING     = 7
    //uint8 RECALLED      = 8
    //uint8 LOST          = 9
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.font = "Ubuntu";
    text.text = "";


    if (!status->status_list.empty()){
      actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
      status_id = goalStatus.status;
    }

    if(status_id == 0){
    //待機中
        text.text = "Pending. Please point your destination.";
    }

    if(status_id == 1){
    //移動中
        text.text = "Auto move.";
    }

    if(status_id == 3){
    //ゴールに到着
        text.text = "Goal is Reached. \n Handover Request. \n Please Operate Manually. \n運転を引き継いで下さい。";
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_state_info_and_task_publisher");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1);

  // subscriber
  ros::Subscriber switch_sub;

  ros::Subscriber move_base_status_sub;
  move_base_status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navStatusCallBack);
  //ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    text.line_width = 1;
    text.text_size = 20;
    text.width = 359;
    text.height = 200;
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
