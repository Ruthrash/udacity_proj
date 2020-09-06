#ifndef LQR_H
#define LQR_H

#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <iterator>
#include <geometry_msgs/PoseStamped.h>



struct CmdVel
{
	double v; 
	double omega; 
};

class LQR
{
public:
	LQR();
	~LQR();
	/**@LQR given the reference to the initial pose of current horizon
	*/
	void LQRControl(const std::vector<geometry_msgs::PoseStamped>::const_iterator &current_it);

	//void Linearize(const geometry_msgs::PoseStamped &pose_, const nav_msgs::Path &reference_path);

protected: 
	std::vector<CmdVel> cmds_;
	int time_window;
	int state_dimension_length;
	int input_dimension_length; 

private:
	double sampling_period;
	Eigen::MatrixXd Q;//weight matrix
	Eigen::MatrixXd R;//Weight matrix
	
	//used for linearization
	Eigen::MatrixXd GetAMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &reference_pose); 
	Eigen::MatrixXd GetBMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &reference_pose);

	//used for path tracking cost function 
	Eigen::MatrixXd GetPMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &reference_pose); 
	Eigen::MatrixXd GetKMatrix(const std::vector<geometry_msgs::PoseStamped>::const_iterator &reference_pose);


};


#endif 