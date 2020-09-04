#ifndef LQR_H
#define LQR_H

#include <Eigen/Dense>
#include <nav_msgs/Path.h>
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
	CmdVel LQRCommand();
	void Linearize(geometry_msgs::PoseStamped pose_, nav_msgs::Path reference_path);

private: 
	CmdVel cmd_vel_; 
	int time_window;

};


#endif 