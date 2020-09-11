#ifndef LQR_H
#define LQR_H

#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <iterator>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>



struct CmdVel
{
	double v; 
	double omega; 
};

class LQR
{
public:
	LQR(const ros::NodeHandle &node_);
	LQR();
	~LQR();
	/**@LQR given the reference to the initial pose of current horizon
	*/
	CmdVel LQRControl(const std::vector<geometry_msgs::PoseStamped>::const_iterator &current_it, const geometry_msgs::PoseStamped &current_pose);

	//void Linearize(const geometry_msgs::PoseStamped &pose_, const nav_msgs::Path &reference_path);

protected: 
	std::vector<CmdVel> cmds_;
	int time_window;
	int state_dimension_length;
	int input_dimension_length; 
	double GetGoalDistance();
	nav_msgs::Path receding_horiz_path; 

private:
	double sampling_period;
	Eigen::MatrixXd Q;//weight matrix
	Eigen::MatrixXd R;//Weight matrix
	
	//used for linearization
	Eigen::MatrixXd GetAMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &end_of_horizon , int steps_to_go); 
	Eigen::MatrixXd GetBMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &end_of_horizon , int steps_to_go);

	//used for path tracking cost function 
	Eigen::MatrixXd GetPMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &K, const Eigen::MatrixXd &prev_P); 
	Eigen::MatrixXd GetKMatrix(const Eigen::MatrixXd &A , const Eigen::MatrixXd &B , const Eigen::MatrixXd &prev_P);

	//get heading angle/yaw from quartenion
	double GetYaw(const std::vector<geometry_msgs::PoseStamped>::const_iterator &reference_pose);
	double GetYaw(const geometry_msgs::PoseStamped& current_pose);
	nav_msgs::Path GetRecedingHorizon(const std::vector<Eigen::VectorXd> &predicted_path);
};


#endif 