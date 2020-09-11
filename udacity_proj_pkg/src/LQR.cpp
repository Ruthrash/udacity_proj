#include "udacity_proj_pkg/LQR.h"


LQR::LQR() : time_window{50}, sampling_period{1} , state_dimension_length{3}, input_dimension_length{2}
{
	//cmds_.resize(time_window + 1);//including timewindow from current pose
	Eigen::Vector3d dummy_Q ;
	dummy_Q << 30,30,150;
	Q = dummy_Q.asDiagonal();
	Eigen::Vector2d dummy_R ;
	dummy_R << 40,160;
	R = dummy_R.asDiagonal();
}

LQR::LQR(const ros::NodeHandle &node_)
{
	
}


LQR::~LQR()
{

}

CmdVel LQR::LQRControl(const std::vector<geometry_msgs::PoseStamped>::const_iterator &current_it, const geometry_msgs::PoseStamped &current_pose)
{
	std::vector<Eigen::VectorXd> predicted_path; 
	Eigen::MatrixXd prev_P;//closed loop cost function's weight matrix 
	//Eigen::MatrixXd K;//state feedback gain matrix
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd K;
	Eigen::MatrixXd P;

	std::vector<geometry_msgs::PoseStamped>::const_iterator end_of_horizon_it = current_it + LQR::time_window; 
	prev_P = Q; //std::vector<CmdVel> cmds_ =  zeroes; 
	CmdVel end_of_horizon_cmd{0.0,0.0};
	cmds_.push_back(end_of_horizon_cmd);
	for (int i = 1 ; i <= LQR::time_window; i++ )//i is steps to go in LQR
	{
		//Linearized around the pose to go(n - (i-1))
		A = GetAMatrix(end_of_horizon_it , i - 1);
		B = GetBMatrix(end_of_horizon_it , i - 1);
		K = GetKMatrix(A , B , prev_P);
		P = GetPMatrix(A , B , K , prev_P); 
		prev_P = P;

		Eigen::VectorXd reference_state_vec(state_dimension_length); 
		reference_state_vec <<  (end_of_horizon_it - (i -1) )->pose.position.x,
								(end_of_horizon_it - (i -1) )->pose.position.y,
		  						GetYaw(end_of_horizon_it - (i -1) );
		Eigen::VectorXd reference_cmd_vec(input_dimension_length); 
		reference_cmd_vec <<cmds_[i-1].v,
							cmds_[i-1].omega;
									
		Eigen::VectorXd current_state_vec(state_dimension_length);
		current_state_vec <<(end_of_horizon_it - i)->pose.position.x, 
							(end_of_horizon_it - i)->pose.position.y, 
							GetYaw(end_of_horizon_it - i);
		Eigen::VectorXd current_cmd_vec(input_dimension_length);
		current_cmd_vec = reference_cmd_vec + K * (current_state_vec - reference_state_vec);

		CmdVel current_cmd; current_cmd.v = current_cmd_vec[0]; current_cmd.omega = current_cmd_vec[1];
		//std::cout<<"command"<<current_cmd.v<<", "<<current_cmd.omega<<"\n";
		Eigen::VectorXd predicted_pose_vec(state_dimension_length);
		predicted_pose_vec = A*current_state_vec + B*current_cmd_vec;

		predicted_path.push_back(predicted_pose_vec);

		cmds_.push_back(current_cmd); 
		std::cout<<"Size="<<cmds_.size()<<"\n";
	}
	//Get matrices for linearized model around the pose to go X(n - (i-1))
	A = GetAMatrix(end_of_horizon_it , LQR::time_window);
	B = GetBMatrix(end_of_horizon_it , LQR::time_window);
	K = GetKMatrix(A , B , prev_P);
	P = GetPMatrix(A , B , K , prev_P); 

	Eigen::VectorXd reference_state_vec(state_dimension_length); 
	reference_state_vec <<(end_of_horizon_it - LQR::time_window )->pose.position.x , (end_of_horizon_it -  LQR::time_window )->pose.position.y , GetYaw(end_of_horizon_it -  LQR::time_window );
	Eigen::VectorXd reference_cmd_vec(input_dimension_length); 
	reference_cmd_vec << cmds_[time_window].v , cmds_[time_window].omega;

	Eigen::VectorXd current_state_vec(state_dimension_length);

	current_state_vec << current_pose.pose.position.x , current_pose.pose.position.y , GetYaw(current_pose);
	Eigen::VectorXd current_cmd_vec(input_dimension_length);

	current_cmd_vec = reference_cmd_vec + K * (current_state_vec - reference_state_vec);
	CmdVel current_cmd; current_cmd.v = current_cmd_vec[0]; current_cmd.omega = current_cmd_vec[1];

	Eigen::VectorXd predicted_pose_vec(state_dimension_length);
	predicted_pose_vec = A*current_state_vec + B*current_cmd_vec;

	predicted_path.push_back(predicted_pose_vec);
	predicted_path.push_back(current_state_vec);
	receding_horiz_path = GetRecedingHorizon(predicted_path);
	
	cmds_.push_back(current_cmd);
	//std::cout<<"command"<<current_cmd.v<<", "<<current_cmd.omega<<"\n";


	cmds_.clear();
	return current_cmd;//return command for current time step 
}

Eigen::MatrixXd LQR::GetAMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &end_of_horizon , int steps_to_go)
{
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(state_dimension_length,state_dimension_length);
	//double yaw = GetYaw(end_of_horizon - steps_to_go);
	
	//A(0,2) = - cmds_[steps_to_go ].v * sin(yaw) * sampling_period;
	//A(1,2) = cmds_[steps_to_go].v * cos(yaw) * sampling_period;
	return A; 
}

Eigen::MatrixXd LQR::GetBMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &end_of_horizon , int steps_to_go)
{
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(state_dimension_length, input_dimension_length);
	double yaw = GetYaw(end_of_horizon - steps_to_go);	

	B(0,0) = cos(yaw) * sampling_period;
	B(1,0) = sin(yaw) * sampling_period;
	B(2,1) = sampling_period;
	return B; 
}

Eigen::MatrixXd LQR::GetKMatrix(const Eigen::MatrixXd &A , const Eigen::MatrixXd &B , const Eigen::MatrixXd &prev_P)
{	
	Eigen::MatrixXd K = LQR::R + B.transpose() * prev_P * B ; 
	K = - K.inverse(); 
	K = K * B.transpose() * prev_P * A; 
	return K ; 
}

Eigen::MatrixXd LQR::GetPMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &K, const Eigen::MatrixXd &prev_P)
{
	Eigen::MatrixXd temp = A + B * K;
	Eigen::MatrixXd	P = LQR::Q + K.transpose() * LQR::R * K + temp.transpose() * prev_P * temp;
	return P; 
}

double LQR::GetYaw(const std::vector<geometry_msgs::PoseStamped>::const_iterator &reference_pose)
{
	tf::Quaternion q(
	(reference_pose)->pose.orientation.x, 
	(reference_pose)->pose.orientation.y,
	(reference_pose)->pose.orientation.z,
	(reference_pose)->pose.orientation.w);
	tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	return yaw;

}

double LQR::GetYaw(const geometry_msgs::PoseStamped& current_pose)
{
	tf::Quaternion q(
	current_pose.pose.orientation.x, 
	current_pose.pose.orientation.y,
	current_pose.pose.orientation.z,
	current_pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	return yaw;

}
/*
double LQR::GetGoalDistance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped&p2)
{	
	double dist = pow(p1.pose.position.x - p2.pose.position.x , 2) + pow(p1.pose.position.y - p2.pose.position.y , 2);
	return sqrt(dist);

}*/

nav_msgs::Path LQR::GetRecedingHorizon(const std::vector<Eigen::VectorXd> &predicted_path)
{
	nav_msgs::Path receding_horiz;
	geometry_msgs::PoseStamped pose_; 
	for(int i = predicted_path.size()-1 ; i >=0 ; --i)
	{
		pose_.header.frame_id = "/odom";
		pose_.header.stamp = ros::Time::now();
		pose_.pose.position.x = predicted_path[i][0];
		pose_.pose.position.y = predicted_path[i][1];

		tf::Matrix3x3 dummy_mat;
		dummy_mat.setEulerYPR(predicted_path[i][2], 0.0, 0.0);
		tf::Quaternion q_;
		dummy_mat.getRotation(q_);
		pose_.pose.orientation.x = q_.getX();
		pose_.pose.orientation.y = q_.getY();
		pose_.pose.orientation.z = q_.getZ();
		pose_.pose.orientation.w = q_.getW();

		receding_horiz.poses.push_back(pose_);
	}
	receding_horiz.header = pose_.header;
	return receding_horiz;
}