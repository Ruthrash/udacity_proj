#ifndef GAZEO_ROS_H
#define GAZEBO_ROS_H

#include <fstream>
#include <iostream>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>


class GazeboROS{

public:
	GazeboROS();
	GazeboROS(ros::NodeHandle &node);
	~GazeboROS();
	void GetRobotPath();
	void PublishRobotPath();
	void PublishRobotPose();

private: 
	gazebo_msgs::GetModelState robot_state;
	ros::ServiceClient robot_state_client;
	nav_msgs::Path robot_path;
	geometry_msgs::PoseStamped prev_pose;
	ros::Publisher robot_path_pub;
	ros::Publisher robot_pose_pub;
	double Distance(const geometry_msgs::PoseStamped &p1 , const geometry_msgs::PoseStamped &p2 );
	std::string file_name;

};

#endif