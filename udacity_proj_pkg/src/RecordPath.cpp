#include "udacity_proj_pkg/GazeboROS.h"
#include "udacity_proj_pkg/PathTracker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle node_;
	GazeboROS _gazebo_ros(node_); 
	ros::Rate loop_rate(10);
	ros::service::waitForService("/gazebo/get_model_state");
	bool start_flag = false;
	std::cout<<"storing poses\n";
	while (ros::ok())
	{
		ros::spinOnce();
		_gazebo_ros.GetRobotPath();
	    _gazebo_ros.PublishRobotPath();
		_gazebo_ros.PublishRobotPose();
		loop_rate.sleep();
	}
    return 0; 
	
}

