#include <iostream>
#include "udacity_proj_pkg/GazeboROS.h"
#include "udacity_proj_pkg/PathTracker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle node_;
	//GazeboROS _gazebo_ros(node_); 
	std::string file_name = "/home/ruthz/Desktop/path.txt";
	PathTracker path_tracker(file_name, node_);
	ros::Rate loop_rate(10);
	ros::service::waitForService("/gazebo/get_model_state");
	while (ros::ok())
	{
		ros::spinOnce();
		//_gazebo_ros.GetRobotPath();
		//_gazebo_ros.PublishRobotPath();
		path_tracker.PublishReferencePath();
		//path_tracker.ClosestPose();
		loop_rate.sleep();
		//std::cout <<"publishin path\n";
	}
	return 0; 
	
}