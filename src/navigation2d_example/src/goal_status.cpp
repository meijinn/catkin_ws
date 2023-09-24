#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9

    if (!status->status_list.empty()){
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
    status_id = goalStatus.status;
    }

    if(status_id==1){
    //移動中
        ROS_INFO("Auto move.");
    }

    if(status_id==3){
    //ゴールに到達・もしくはゴールに到達して待機中。
        ROS_INFO("Goal is Reached. Please Handover.");
    }

    if(status_id==0){
        ROS_INFO("Pending. Please point your destination.");
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_state");

    ros::NodeHandle nh;
    ros::Subscriber switch_sub;

    ros::Subscriber move_base_status_sub;
    move_base_status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navStatusCallBack);
    ros::spin();

    return 0;
}
