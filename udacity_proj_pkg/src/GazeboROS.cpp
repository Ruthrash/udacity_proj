#include "udacity_proj_pkg/GazeboROS.h"

GazeboROS::GazeboROS(ros::NodeHandle &node)
{
	robot_state_client = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	robot_state.request.model_name = "jackal";//_node.param<std::string>("robot_model_name", "");;
	robot_state.request.relative_entity_name = "";
	robot_path_pub = node.advertise<nav_msgs::Path>("/robot_reference_path",1,true);
}

/*
* @brief save recorded path in a text file before destruction
**/
GazeboROS::~GazeboROS()
{	
	std::ofstream poses_file("/home/ruthz/Desktop/path.txt");
	if(poses_file.is_open() && robot_path.poses.size()!=0)
	{
		std::cout<<"Storing recorded path with "<< robot_path.poses.size() <<" poses\n";
		for(int i = 0; i < robot_path.poses.size(); ++i)
		{
			poses_file << robot_path.poses[i].pose.position.x<<" "<<robot_path.poses[i].pose.position.y<<" "<<robot_path.poses[i].pose.position.z<<" "
			<<robot_path.poses[i].pose.orientation.x<<" "<<robot_path.poses[i].pose.orientation.y<<" "
			<<robot_path.poses[i].pose.orientation.z<<" "<<robot_path.poses[i].pose.orientation.w <<"\n";
		}
		poses_file.close();
	}
}

void GazeboROS::GetRobotPath()
{
  std::cout<<"getting poses\n";
  robot_state_client.call(robot_state);
  geometry_msgs::PoseStamped pose_;
  pose_.pose = robot_state.response.pose;
  pose_.header.frame_id = "/odom"; 
  pose_.header.stamp = ros::Time::now();
  robot_path.header = pose_.header;
  robot_path.poses.push_back(pose_);
}

void GazeboROS::PublishRobotPath()
{
	robot_path_pub.publish(robot_path);
}