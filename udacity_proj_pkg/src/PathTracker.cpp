#include "udacity_proj_pkg/PathTracker.h"


PathTracker::~PathTracker()
{
	
}

PathTracker::PathTracker(std::string poses_file_name, ros::NodeHandle &node_) : PathTrackerROS(node_)
{

	GetPath(poses_file_name);//loads path from text file
	std::cout << "Poses file being parsed\n";
	std::cout << "Contains "<<reference_path.poses.size()<<" poses \n";
}

void PathTrackerROS::GetPath(std::string poses_file_name)
{
	std::fstream poses_file(poses_file_name);
	std::cout<<poses_file_name<<"\n";
	if(poses_file.is_open())
	{
		std::string line, word;
		while (std::getline(poses_file, line))
		{
			std::cout<<"getting line \n";
			geometry_msgs::PoseStamped pose_;
			std::istringstream ss(line);
			//std::cout<<line<<"\n";		
			ss >> word; pose_.pose.position.x = stod(word);
			ss >> word; pose_.pose.position.y = stod(word);
			ss >> word; pose_.pose.position.z = stod(word);
			ss >> word; pose_.pose.orientation.x = stod(word);
			ss >> word; pose_.pose.orientation.y = stod(word);
			ss >> word; pose_.pose.orientation.z = stod(word);
			ss >> word; pose_.pose.orientation.w = stod(word);
			pose_.header.frame_id = "odom";
			reference_path.poses.push_back(pose_);
		}
		poses_file.close();
	}
	else
		std::cout<<"Unable to parse file \n";
}

void PathTracker::TrackerInit()
{
	//find closest point in the reference path to current pose and call TrackPath
	geometry_msgs::PoseStamped current_pose = GetCurrentPose();
	std::vector<geometry_msgs::PoseStamped>::const_iterator closest_it = GetClosestPose(current_pose);
	TrackPath(closest_it);
}

void PathTracker::TrackPath(const std::vector<geometry_msgs::PoseStamped>::const_iterator &closest_it)
{

	std::vector<geometry_msgs::PoseStamped>::const_iterator lqr_it = closest_it;
	//takes care of receding horizon 
	while((lqr_it + LQR::time_window) - reference_path.poses.begin() <= reference_path.poses.size())
	{
		CmdVel cmd_ = LQR::LQRControl(lqr_it, GetCurrentPose());
		PathTrackerROS::PublishControlCmd(cmd_);
		++lqr_it;
		ros::Duration(0.5).sleep();
	}	 
	std::cout<<"Done tracking \n";
	//Publish current velocity command
	//publish receding horizon path 
}


std::vector<geometry_msgs::PoseStamped>::const_iterator PathTracker::GetClosestPose(const geometry_msgs::PoseStamped &current_pose)
{
	double min_dist = 1000.0; 
	std::vector<geometry_msgs::PoseStamped>::const_iterator min_it = reference_path.poses.begin(); 
	for(std::vector<geometry_msgs::PoseStamped>::const_iterator iter = reference_path.poses.begin(); iter != reference_path.poses.end(); ++iter)
	{
		double dist = pow(iter->pose.position.x - current_pose.pose.position.x, 2) + pow(iter->pose.position.y - current_pose.pose.position.y, 2) ;
		dist = sqrt(dist);
		if(dist < min_dist)
		{
			min_dist = dist; 
			min_it = iter; 
		}
	}
	return min_it;
}














PathTrackerROS::PathTrackerROS(){}
PathTrackerROS::PathTrackerROS(ros::NodeHandle &node_)
{
	reference_path_pub = node_.advertise<nav_msgs::Path>("/robot_reference_path", 1, true);
	cmd_vel_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

	//gazebo
    robot_state_client = node_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	robot_state.request.model_name = "/";//_node.param<std::string>("robot_model_name", "");;
	robot_state.request.relative_entity_name = "";
} 
PathTrackerROS::~PathTrackerROS(){}

void PathTrackerROS::PublishReferencePath()
{
	reference_path.header.stamp = ros::Time::now();
	reference_path.header.frame_id = "odom";
	reference_path_pub.publish(reference_path);
	//std::cout<<"Publishing Path\n";
}
void PublishTrackedPath()
{

}

geometry_msgs::PoseStamped PathTrackerROS::GetCurrentPose()
{
	robot_state_client.call(robot_state);
	geometry_msgs::PoseStamped pose_;
	pose_.pose = robot_state.response.pose;
	pose_.header.frame_id = "/odom"; 
	pose_.header.stamp = ros::Time::now();
	return pose_;
}

void PathTrackerROS::PublishControlCmd(CmdVel cmd_)
{
	geometry_msgs::Twist cmd; 
	cmd.linear.x = cmd_.v ; 
	cmd.angular.z = cmd_.omega; 
	cmd_vel_pub.publish(cmd); 
	std::cout<<"Publishing (v,omeag) "<< cmd_.v <<", "<<cmd_.omega<<"\n";

}