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
	while(GetGoalDistance() >  0.05 )
	{
		std::vector<geometry_msgs::PoseStamped>::const_iterator closest_it = GetClosestPose(current_pose);
		TrackPath(closest_it);
		current_pose = GetCurrentPose();
		if(PathTracker::end_flag)
			break;
	}
	std::cout<<"done tracking\n";

	
}

void PathTracker::TrackPath(const std::vector<geometry_msgs::PoseStamped>::const_iterator &closest_it)
{
	//starts a thread that uses the computed commands in the horizon to predict the path in the current horizon. 
	auto waiting_thread = std::async(std::launch::async,&PathTracker::WaitAndPublishRecedingHorizon, this);
	
	CmdVel cmd_;
	//optimizes over one time horizon.
	if(closest_it-reference_path.poses.begin() + 1.5*LQR::time_window  >= reference_path.poses.size()-1)
	{
		LQR::time_window = LQR::time_window/2; //close to the end of the path, reduce time_window
		cmd_ = LQR::LQRControl(closest_it, GetCurrentPose(), 0 );
		PathTrackerROS::PublishControlCmd(cmd_);
		PathTrackerROS::PublishCurrentPose();
		PathTrackerROS::PublishTrackedPath();
		cmds_.clear();

		if(closest_it-reference_path.poses.begin() + LQR::time_window  >= reference_path.poses.size()-1)//during last reduced time window run open loop
		{
			cmd_ = LQR::LQRControl(closest_it, GetCurrentPose(), 0);
			PathTrackerROS::PublishControlCmd(LQR::cmds_, double(time_window/sampling_period));
			cmds_.clear();
			PathTracker::end_flag = true;
			return;
		}
	}
	else
		cmd_ = LQR::LQRControl(closest_it, GetCurrentPose(), 0 );
		PathTrackerROS::PublishControlCmd(cmd_);
		PathTrackerROS::PublishCurrentPose();
		PathTrackerROS::PublishTrackedPath();
		cmds_.clear();	
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
	return min_it;
}

double PathTracker::GetGoalDistance()
{
	//calculates  eucl distance not absolute path length  
	geometry_msgs::PoseStamped current_pose = GetCurrentPose();
	double distance = pow(current_pose.pose.position.x - (reference_path.poses.end()-1)->pose.position.x , 2 ) +
					 pow(current_pose.pose.position.y - (reference_path.poses.end()-1)->pose.position.y , 2 ) ;
					  //abs(GetYawFromQuart(current_pose) - GetYawFromQuart(*(reference_path.poses.end()-1)) );
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
	ros::service::waitForService("/jackal_velocity_controller/set_parameters");
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


void PathTrackerROS::PublishControlCmd(std::vector<CmdVel> cmds_, double rate)
{
	for(int i = cmds_.size() - 1; i >=0 ;--i )
	{
		geometry_msgs::Twist cmd; 
		cmd.linear.x = cmds_[i].v ; 
		cmd.angular.z = cmds_[i].omega; 
		if(abs(cmd.linear.x >= 2))
			cmd.linear.x = 2*cmd.linear.x/abs(cmd.linear.x);
		if(abs(cmd.angular.z >= 2))
			cmd.angular.z = 2*cmd.angular.z/abs(cmd.angular.z);

		cmd_vel_pub.publish(cmd); 
		std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0*rate)));
		std::cout<<int(1000.0*rate) <<"\n";
		PathTrackerROS::PublishTrackedPath();
		PathTrackerROS::PublishCurrentPose();
	}

}


void PathTrackerROS::PublishCurrentPose()
{
	current_pose_pub.publish(PathTrackerROS::GetCurrentPose());
} 


void PathTrackerROS::PublishRecedingHorizon(const nav_msgs::Path & receding_horiz_path)
{
	receding_horiz_pub.publish(receding_horiz_path);
}

void PathTracker::WaitAndPublishRecedingHorizon()
{
	nav_msgs::Path predicted_path;
	while(true)
    {
		predicted_path = LQR::message_queue.Receive();
        if(predicted_path.poses.size()!=0)
            break;
    }

	PathTrackerROS::PublishRecedingHorizon(predicted_path);
	//publish the stuff
}

