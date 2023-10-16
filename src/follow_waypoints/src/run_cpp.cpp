#include <string>
#include <fstream>
#include <angles/angles.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Note : This code is based on ROS Melodic and cmake version 3.0.2

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool get_localized_pose(tf::TransformListener &listener, geometry_msgs::Pose2D &result, const double time_limit=10.0, const std::string &target="map", const std::string &source="base_link")
{
    try
    {
        tf::StampedTransform transform;
        listener.waitForTransform(target, source, ros::Time(0), ros::Duration(time_limit) );
        listener.lookupTransform(target, source, ros::Time(0), transform);
        result.x = transform.getOrigin().x();
        result.y = transform.getOrigin().y();
        result.theta = tf::getYaw(transform.getRotation());
        return true;
    }
    catch (tf::TransformException ex) 
    {
        ROS_ERROR("%s",ex.what());
        return false;
    }
}

std::vector<std::string> split(const std::string &str, const char del) {
    int first = 0;
    int last = str.find_first_of(del);
    std::vector<std::string> result;
    while (first < str.size())
    {
        std::string subStr(str, first, last - first);
        result.push_back(subStr);
        first = last + 1;
        last = str.find_first_of(del, first);
        if (last == std::string::npos) {
            last = str.size();
        }
    }
    return result;
}

std::vector<std::vector<std::string>> getCsv(const std::string &filepath, const int head_lines=0)
{   
    std::ifstream read_file;
    read_file.open(filepath, std::ios::in);
    if(!read_file){
        std::vector<std::vector<std::string>> points;
        return points;
    }
    std::string buffer;

    // Ignore the number of head lines
    for (int i=0; i<head_lines; i++)
    {
        getline(read_file, buffer);
        if(read_file.eof()) break;
    }

    // Make vector
    std::vector<std::vector<std::string>> points;
    while(std::getline(read_file, buffer))
    {
        if(buffer.size() == 0) break;
        std::vector<std::string> data;
        data = split(buffer, ',');
        points.push_back(data);
    }
    return points;
}

std::vector<move_base_msgs::MoveBaseGoal> getWaypoints(const std::vector<std::vector<std::string>> &points, const std::string &frame_id="map")
{
    move_base_msgs::MoveBaseGoal wp;
    std::vector<move_base_msgs::MoveBaseGoal> waypoints;
    for(int i=0; i<points.size(); i++)
    {
        wp.target_pose.header.frame_id = frame_id;
        wp.target_pose.header.stamp = ros::Time::now();
        wp.target_pose.pose.position.x = std::stod(points[i][1]);
        wp.target_pose.pose.position.y = std::stod(points[i][2]);
        tf::Quaternion q;
        q.setRPY(0,0,0);
        q = q.normalize();
        wp.target_pose.pose.orientation.x = q.x();
        wp.target_pose.pose.orientation.y = q.y();
        wp.target_pose.pose.orientation.z = q.z();
        wp.target_pose.pose.orientation.w = q.w();
        waypoints.push_back(wp);
    }
    return waypoints;
}

double getEuclideanDistance(const double x0, const double y0, const double x1, const double y1)
{
    return sqrt(pow((x1-x0),2)+pow((y1-y0),2));
}

void sendGoals(const std::vector<move_base_msgs::MoveBaseGoal> &goals, const double duration, const double distance_tolerance)
{
    geometry_msgs::Pose2D pose;
    tf::TransformListener listener;

    // Action Client
    MoveBaseClient ac("move_base", true);

    // Wait for action server
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Come up move_base action server");

    ROS_INFO("Send goals %ld times", goals.size());

    for (int i=0; i<goals.size(); i++)
    {
        ROS_INFO("Sending goal");
        ac.sendGoal(goals[i]);
        double target_pose_x = goals[i].target_pose.pose.position.x;
        double target_pose_y = goals[i].target_pose.pose.position.y;

        ROS_INFO("Goal position x=%.3f, y=%.3f", target_pose_x, target_pose_y);
        
        if (!distance_tolerance > 0.0)
        {
            ac.waitForResult();
            ROS_INFO("Waiting for %.2f sec...", duration);
        }
        else{
            if (get_localized_pose(listener, pose))
            {
                ROS_INFO("[Localized pose] (x, y, theta) = (%.2f, %.2f, %.2f)", pose.x, pose.y, angles::to_degrees(pose.theta));
            }

            double distance = getEuclideanDistance(target_pose_x, target_pose_y, pose.x, pose.y);

            ROS_INFO("Remain distance:%.3f", distance);

            while (distance > distance_tolerance)
            {
                if (get_localized_pose(listener, pose))
                {
                    // ROS_INFO("[Localized pose] (x, y, theta) = (%.2f, %.2f, %.2f)", pose.x, pose.y, angles::to_degrees(pose.theta));
                    distance = getEuclideanDistance(target_pose_x, target_pose_y, pose.x, pose.y);
                    // ROS_INFO("Remain distance:%.3f", distance);
                }

                ros::Duration(0.1).sleep();
            }
        }
        // Cancel Goal
        ac.cancelGoal();
        ROS_INFO("Reached distance tolerance");
    }
    // Done every goal
    ROS_INFO("Done every goal");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_cpp");

    // Get params
    ros::NodeHandle pnh("~");
    double duration = 0.0;
    pnh.getParam("duration", duration);
    double distance_tolerance = 1.0;
    pnh.getParam("distance_tolerance", distance_tolerance);
    std::string pkg_name = "follow_waypoints";
    std::string pkg_path = ros::package::getPath(pkg_name);
    std::string path_to_waypoints = pkg_path + "/csv/waypoints.csv";
    pnh.getParam("path_to_waypoints", path_to_waypoints);

    // Show params 
    ROS_WARN("[Params] duration:%.2f, distance_tolerance:%.2f, path_to_waypoints:%s", duration, distance_tolerance, path_to_waypoints.c_str());

    // Get point from csv
    std::vector<std::vector<std::string>> points = getCsv(path_to_waypoints, 1);

    // Get waypoints from points
    std::vector<move_base_msgs::MoveBaseGoal> goals = getWaypoints(points);

    // Send goals
    sendGoals(goals, duration, distance_tolerance);

    return 0;
}