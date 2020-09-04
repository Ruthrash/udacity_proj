#ifndef GAZEO_ROS_H
#define GAZEBO_ROS_H

#include <fstream>
#include <iostream>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>


class GazeboROS{

public:
	GazeboROS();
	GazeboROS(ros::NodeHandle &node);
	~GazeboROS();
	void GetRobotPath();
	void PublishRobotPath();

private: 
	gazebo_msgs::GetModelState robot_state;
	ros::ServiceClient robot_state_client;
	nav_msgs::Path robot_path; 
	ros::Publisher robot_path_pub;

};

#endif