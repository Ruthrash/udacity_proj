#include "udacity_proj_pkg/LQR.h"

LQR::LQR() : time_window{25}, sampling_period{1.0} 
{
	cmds_.resize(time_window);
	for(int i = 0; i < cmds_.size(); ++i)
	{
		cmds_[i].v = 0.0;
		cmds_[i].omega = 0.0;
	}
	Eigen::MatrixXd dummy_Q ;
	dummy_Q << 1,1,4;
	Q = dummy_Q.array().sqrt().matrix().asDiagonal();
	Eigen::MatrixXd dummy_R ;
	dummy_R << 1,3;
	R = dummy_R.array().sqrt().matrix().asDiagonal();
}

LQR::~LQR()
{

}

void LQR::LQRControl(const std::vector<geometry_msgs::PoseStamped>::const_iterator &current_it)
{
	Eigen::MatrixXd P;//closed loop cost function's weight matrix 
	Eigen::MatrixXd K;//state feedback gain matrix 
	P = Q; //std::vector<CmdVel> cmds_ =  zeroes; 

	for (int i = 0 ; i < LQR::time_window; i++ )
	{
		//calculate A, B 

	}

}
