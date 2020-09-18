#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <iterator>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include "udacity_proj_pkg/LQR.h" 

//gazebo
#include <gazebo_msgs/GetModelState.h>


/*
* @brief encapsulates ROS related stuff for path tracking
*/
class PathTrackerROS
{
public:
	PathTrackerROS();
	~PathTrackerROS();
	PathTrackerROS(ros::NodeHandle &node_);
	void PublishReferencePath();
	void PublishTrackedPath();
	void PublishCurrentPose();
	void GetPath(std::string poses_file_name);
	void PublishRecedingHorizon(const nav_msgs::Path &receding_horiz_path);
	void PublishControlCmd(CmdVel cmd_);
	void PublishControlCmd(std::vector<CmdVel> cmds_, double rate);

protected:
	nav_msgs::Path reference_path; 
	nav_msgs::Path tracked_path;
	geometry_msgs::Twist cmd_vel;
	geometry_msgs::PoseStamped GetCurrentPose();
	



private:
	ros::Publisher reference_path_pub;
	ros::Publisher tracked_path_pub;
	ros::Publisher receding_horiz_pub;
	ros::Publisher cmd_vel_pub; 
	ros::Publisher current_pose_pub; 
	//gazebo for ground truth

	gazebo_msgs::GetModelState robot_state;
	ros::ServiceClient robot_state_client;
	
};


/*
* @brief encapsulates the complete PathTracker object containing LQR path tracking controller and ROS related stuff through inheritance 
*/

class PathTracker : public LQR, public PathTrackerROS
{
public:
	PathTracker();
	PathTracker(std::string poses_file_name,  ros::NodeHandle &node_) ;
	~PathTracker();
	/* @brief Initializes the tracker 
	*/
	void TrackerInit(); 


private: 

	/* @brief Runs the LQR algorithm given the iterator rerence of closest point in the reference path 
	*/
	void TrackPath(const std::vector<geometry_msgs::PoseStamped>::const_iterator &closest_it); 

	/* @brief Finds closest pose in the path to current pose and returns its index  
	*/
	std::vector<geometry_msgs::PoseStamped>::const_iterator GetClosestPose(const geometry_msgs::PoseStamped &current_pose); 

	/* @brief Finds distance from current pose to the goal point 
	*/
	double GetGoalDistance(); 	
    /* @brief Finds yaw from quartenion
	*/
	double GetYawFromQuart(const geometry_msgs::PoseStamped &msg);
	
	void WaitAndPublishRecedingHorizon();
	bool end_flag{false}; 
	
};
#endif