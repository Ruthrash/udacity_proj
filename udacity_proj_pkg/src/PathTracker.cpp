#include "udacity_proj_pkg/PathTracker.h"


PathTracker::~PathTracker()
{
	
}

PathTracker::PathTracker(std::string poses_file_name, ros::NodeHandle &node_) : PathTrackerROS(node_),LQR(node_)
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
	//while(LQR::Distance(current_pose,*(reference_path.poses.end()-1)) >  0.5 )
	while(GetGoalDistance() >  0.5 )
	{
		std::vector<geometry_msgs::PoseStamped>::const_iterator closest_it = GetClosestPose(current_pose);
		TrackPath(closest_it);
		current_pose = GetCurrentPose();
	}
	
}

void PathTracker::TrackPath(const std::vector<geometry_msgs::PoseStamped>::const_iterator &closest_it)
{

	std::vector<geometry_msgs::PoseStamped>::const_iterator lqr_it = closest_it;
	std::cout<<"closest"<<closest_it-reference_path.poses.begin()<<"\n";
	//takes care of receding horizon 

	//while((lqr_it + LQR::time_window) - reference_path.poses.begin() <= reference_path.poses.size())
	//{
		CmdVel cmd_ = LQR::LQRControl(lqr_it, GetCurrentPose(),closest_it-reference_path.poses.begin());//for one time horizon
		PathTrackerROS::PublishControlCmd(cmd_);
		//PathTrackerROS::PublishCurrentPose();
		PathTrackerROS::PublishTrackedPath();
		PathTrackerROS::PublishRecedingHorizon(LQR::receding_horiz_path);
		//++lqr_it;
	//}	 
	//std::cout<<"Done tracking \n";
	//Publish current velocity command
	//publish receding horizon path 
}


std::vector<geometry_msgs::PoseStamped>::const_iterator PathTracker::GetClosestPose(const geometry_msgs::PoseStamped &current_pose)
{
	double min_dist = 100000.0; 
	std::vector<geometry_msgs::PoseStamped>::const_iterator min_it = reference_path.poses.begin(); 
	for(std::vector<geometry_msgs::PoseStamped>::const_iterator iter = reference_path.poses.begin(); iter != reference_path.poses.end(); ++iter)
	{

		
		double dist = pow(iter->pose.position.x - current_pose.pose.position.x, 2) + 
					pow(iter->pose.position.y - current_pose.pose.position.y, 2) + 
					abs(GetYawFromQuart(*iter) - GetYawFromQuart(current_pose) );
		dist = sqrt(dist);
		if(dist < min_dist)
		{
			min_dist = dist; 
			min_it = iter; 
		}
	}
	std::cout<<"Min dist "<<min_dist<<"\n";
	std::cout<<"Closest  "<<int(min_it - reference_path.poses.begin())<<"\n";
	return min_it;
}

double PathTracker::GetGoalDistance()
{
	geometry_msgs::PoseStamped current_pose = GetCurrentPose();
	double distance = pow(current_pose.pose.position.x - (reference_path.poses.end()-1)->pose.position.x , 2 ) +
					 pow(current_pose.pose.position.y - (reference_path.poses.end()-1)->pose.position.y , 2 );
	distance = sqrt(distance);
	return distance; 
}

double PathTracker::GetYawFromQuart(const geometry_msgs::PoseStamped &msg)
{
		tf::Quaternion q(msg.pose.orientation.x,
						msg.pose.orientation.y,
						msg.pose.orientation.z,
						msg.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		return yaw; 
}




PathTrackerROS::PathTrackerROS(){}
PathTrackerROS::PathTrackerROS(ros::NodeHandle &node_)
{	
	reference_path_pub = node_.advertise<nav_msgs::Path>("/reference_path", 1, true);
	current_pose_pub = node_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1, true);
	cmd_vel_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	tracked_path_pub = node_.advertise<nav_msgs::Path>("/tracked_path", 1, true);
	receding_horiz_pub = node_.advertise<nav_msgs::Path>("/receding_horizon_path", 1, true); 

	//gazebo
    robot_state_client = node_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	robot_state.request.model_name = "jackal";//_node.param<std::string>("robot_model_name", "");;
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
void PathTrackerROS::PublishTrackedPath()
{
	tracked_path.poses.push_back(GetCurrentPose());
	tracked_path.header.stamp = ros::Time::now();
	tracked_path.header.frame_id = "odom";
	tracked_path_pub.publish(tracked_path);

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

	if(abs(cmd_.v >= 2))
		cmd.linear.x = 2*cmd_.v/abs(cmd_.v);
	if(abs(cmd_.omega >= 2))
		cmd.angular.z = 2*cmd_.omega/abs(cmd_.omega);

	cmd_vel_pub.publish(cmd); 
	//std::cout<<"Publishing (v,omeag) "<< cmd_.v <<", "<<cmd_.omega<<"\n";

}

void PathTrackerROS::PublishCurrentPose()
{
	current_pose_pub.publish(PathTrackerROS::GetCurrentPose());
} 


void PathTrackerROS::PublishRecedingHorizon(const nav_msgs::Path & receding_horiz_path)
{
	receding_horiz_pub.publish(receding_horiz_path);
}



