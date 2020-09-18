## Description

A ROS Package for running an MPC Path Tracking controller(LQR in the backend) for a differential-drive robot tested in Gazebo simulation. Capstone project for Udacity C++ Nanodegree. Details on how to use the ROS package is mentioned here. You can also find below a brief description of the LQR controller and some information on how to tune it. This project was built and tested in Ubuntu 16.04 and ROS Kinetic + Gazebo7. 

## Path Tracking Control
Given a reference path, path tracking control is an algorithm to compute input commands for the robot to ensure it traverses the reference path as close as possible. Contrary to trajectory tracking, this algorithm doesn't account for the time it takes to traverse the path and only try to minimize the offset between the reference path and the actual traversed path. 

Given the current state and a "goal" state of the system, Model Predictive Control(MPC) is an control paradigm to compute input commands to drive the system to the goal state as below, 

-  A model of the system is used to numerically solve an open-loop optimization problem at each step of a look-ahead prediction horizon. 
- Even though we have the control commands for all time steps in the current time horizon, only the first control command is executed.
- During the next time step, current state of the system is measured and the algorithm starts again, running till the system reaches and stays at the "goal" state.  

In this project, the optimization algorithm used in the backend for each time step is a Linear Quadratic Regulator. This project uses a neat closed form solution to this optimization problem(look at References). Although, the differential-drive model(our system) is non-linear, we approximate it as a linear system at each time step when the LQR is run. Pseudocode for the MPC and LQR can be found below. 



```
current_control_command func MPC(current_pose, reference_path)
    WHILE DISTANCE(current_pose, ENDOF(reference_path) < THRESHOLD) DO:
        commands = OPTIMIZECURRENTHORIZON(current_pose, CURRENTHORIZON(reference_path))   
        current_pose = GETCURRENTPOSE() 
    end WHILE
    return commands[0]
end func
```

```
commands func OPTIMIZECURRENTHORIZON(current_pose, current_horizon)
    reference_input = {0}
    array commands
    FOR goal_pose in (END(current_horizon), START(current_horizon))
        PUSHBACK(commands, LQR(prev_pose, goal_pose, reference_input))
    end FOR
    PUSHBACK(commands, LQR(current_pose, START(current_horizon), reference_input))
    return commands
end func    
```
```
control_command func LQR(current_pose, goal_pose, reference_input)
    LINEARIZE(goal_pose, reference_input)
    COMPUTE K(gain matrix)
    COMPUTE (control_command, K)
    return control_command
end func
```


## Dependancies
- ROS Kinetic Kame 
- Gazebo 7
- C++11 STL
- Eigen
- Jackal Simulation
- turtlebot_teleop(Optional, teleoperation to record reference path)
- rqt_multiplot(Optional, for visualization)

## Installation
Install dependancies 
##### Eigen

```bash
cd 
git clone https://gitlab.com/libeigen/eigen.git
cd eigen 
mkdir build 
cd build 
cmake ../
sudo make install
```

##### Clearpath Jackal Simulation
```bash
sudo apt-get install ros-kinetic-jackal-simulator
```

##### turtlebot_teleop
```bash
sudo apt-get install ros-kinetic-turtlebot-teleop
```

catkin_make is used to build the package. We therefore have to follow ROS workspace structure of directory
##### Create a ROS workspace, pull and build the code
```bash
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/src
catkin_init_workspace 
git clone https://github.com/Ruthrash/udacity_proj
catkin_make_isolated
```
## Usage


##### source current workspace
```bash
cd catkin_ws 
source devel/setup.bash
```

Record Path
```bash
roslaunch udacity_proj_pkg record_path.launch file_name:="/path/to/store/recorded_path.txt" 
```
##### open a new terminal for teleoperation 
```bash
rosrun turtlebot_teleop turtlebot_teleop_key turtlebot_teleop/cmd_vel:=cmd_vel
```
##### use the keys shown in this terminal to control the simulated jackal robot
Track Path
##### configure parameters in config/controller_params.yaml and run
```bash
roslaunch udacity_proj_pkg udacity_project.launch
```

##### This command should open an Rviz GUI window showing the greeen path as the reference path, blue as the true tracked path and red is the predicted path in the current look ahead time horizon


## CodeBase



## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[MIT](https://choosealicense.com/licenses/mit/)