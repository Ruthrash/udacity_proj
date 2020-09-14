#include "udacity_proj_pkg/LQR.h"


LQR::LQR(const ros::NodeHandle &node_) //: time_window{30}, sampling_period{2} , state_dimension_length{3}, input_dimension_length{2}
{
	//cmds_.resize(time_window + 1);//including timewindow from current pose
	/*Eigen::Vector3d dummy_Q ;
	dummy_Q <<70,70,150;
	Q = dummy_Q.asDiagonal();
	Eigen::Vector2d dummy_R ;
	dummy_R << 30,120;
	R = dummy_R.asDiagonal();*/
	
	std::string dummy;
	
	node_.getParam("R_matrix_diag", dummy);
	R = ParseParam::ConvertStringToDiagMatrix(dummy);

	
	node_.getParam("Q_matrix_diag", dummy);
	Q = ParseParam::ConvertStringToDiagMatrix(dummy);
	
	node_.getParam("time_window", dummy);
	time_window = ParseParam::ConvertStringToDouble(dummy);

	node_.getParam("state_dimension_length", dummy);
	state_dimension_length = ParseParam::ConvertStringToDouble(dummy);

	node_.getParam("sampling_period", dummy);
	sampling_period = ParseParam::ConvertStringToDouble(dummy);
		
	node_.getParam("input_dimension_length", dummy);
	input_dimension_length = ParseParam::ConvertStringToDouble(dummy);

	std::cout<<"!!!PARAMSS!!!\n"<<input_dimension_length<<",\n"
							  <<state_dimension_length<<",\n"
							  <<sampling_period<<",\n"
							  <<time_window<<",\n"
							  <<Q<<",\n"
							  <<R<<"\n";

}

LQR::LQR(){}
LQR::~LQR()
{

}

CmdVel LQR::LQRControl(const std::vector<geometry_msgs::PoseStamped>::const_iterator &current_it, const geometry_msgs::PoseStamped &current_pose, 
										const int &closest_idx)
{
	//std::vector<Eigen::VectorXd> predicted_path; 
	Eigen::MatrixXd prev_P;//closed loop cost function's weight matrix 
	//Eigen::MatrixXd K;//state feedback gain matrix
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd K;
	Eigen::MatrixXd P;
	
	//if(closest_idx > )
	std::vector<geometry_msgs::PoseStamped>::const_iterator end_of_horizon_it = current_it + LQR::time_window; 
	prev_P = Q; //std::vector<CmdVel> cmds_ =  zeroes; 
	CmdVel end_of_horizon_cmd{0.0,0.0};
	cmds_.push_back(end_of_horizon_cmd);
	std::vector<Eigen::VectorXd> states_;
	std::vector<Eigen::MatrixXd> A_vec, B_vec; 

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
		states_.push_back(current_state_vec);
		cmds_.push_back(current_cmd); 
		A_vec.push_back(A); B_vec.push_back(B);
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

	states_.push_back(current_state_vec);
	cmds_.push_back(current_cmd);
	A_vec.push_back(A); B_vec.push_back(B);

	//start a thread to compute receding_horiz_path and it also sends the receding horizon path to the message queue
	all_threads.push_back(std::move(std::thread(&LQR::GetPredictedPath,this, states_, cmds_, A_vec , B_vec))   );
	std::cout<<"threads"<<all_threads.size()<<"\n";
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


//this should start in a thread and send to the message queue once done
void LQR::GetPredictedPath(const std::vector<Eigen::VectorXd> &states_, const std::vector<CmdVel> cmds,
													const std::vector<Eigen::MatrixXd> &A_vec, const std::vector<Eigen::MatrixXd> &B_vec )
{
	std::vector<Eigen::VectorXd> predicted_path_eig;
	//std::cout<<"Sizes!!"<<states_.size()<<","<<cmds.size()<<"\n";
	for(int i = 0; i < states_.size(); ++i)
	{
		Eigen::VectorXd predicted_pose_vec(state_dimension_length), current_cmd_vec(input_dimension_length);
		current_cmd_vec << cmds[i+1].v , cmds[i+1].omega;//since cmds_ contains (0,0) input command as well. pushed at line 64
		predicted_pose_vec = A_vec[i] * states_[i] + B_vec[i] * current_cmd_vec;
		predicted_path_eig.push_back(predicted_pose_vec);
	}
	predicted_path_eig.push_back(*(states_.end()-1));

	nav_msgs::Path receding_horiz_path = GetRecedingHorizon(predicted_path_eig);
	message_queue.Send(std::move (receding_horiz_path));
	
}



/* Implementation of class "MessageQueue" */


template <typename T>
T MessageQueue<T>::Receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
    // perform queue modification under the lock
    std::unique_lock<std::mutex> uLock(_mtx);
    _cond.wait(uLock, [this] { return !_queue.empty(); }); // pass unique lock to condition variable

    // remove last vector element from queue
    T msg = std::move(_queue.back());
    _queue.pop_back();

    return msg; // will not be copied due to return value optimization (RVO) in C++
}

template <typename T>
void MessageQueue<T>::Send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    // perform vector modification under the lock
    std::lock_guard<std::mutex> uLock(_mtx);

    // add vector to queue
    //std::cout << "   Message " << msg << " has been sent to the queue" << std::endl;
    //_queue.push_back(std::move(msg));
    _queue.push_back(msg);
    _cond.notify_one(); // notify client after pushing new Vehicle into vector
}



template class MessageQueue<nav_msgs::Path>;