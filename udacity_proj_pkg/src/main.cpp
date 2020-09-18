#include "udacity_proj_pkg/GazeboROS.h"
#include "udacity_proj_pkg/PathTracker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle node_;
	std::string file_name = "/home/ruthz/Desktop/path.txt";
	node_.getParam("recorded_path_file", file_name);
	std::cout<<file_name<<"\n";
	PathTracker path_tracker(file_name, node_);
	ros::Rate loop_rate(10);
	ros::service::waitForService("/jackal_velocity_controller/set_parameters");
	//ros::service::waitForService("/gazebo/get_model_state");
	bool start_flag = false;
	ROS_INFO("!!!!!!!Initialized tracker!!!!!!!");
	//std::cout<<"going to track\n";
	while (ros::ok())
	{
		ros::spinOnce();
		path_tracker.PublishReferencePath();
		if(!start_flag)
		{
			path_tracker.TrackerInit();
			start_flag = true;
		}

		loop_rate.sleep();

	}
	return 0; 
	
}

/*to do: 
get parameters for tuning in YAML file
and tune
publish, reference and tracked path 

*/