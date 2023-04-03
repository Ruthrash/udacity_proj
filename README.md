## Video 

[![Video](https://youtu.be/QZjlcsgegM4?list=TLGGnr1uvdBcjNUwMzA0MjAyMw/0.jpg)](https://youtu.be/QZjlcsgegM4?list=TLGGnr1uvdBcjNUwMzA0MjAyMw)

## Description

A ROS Package for running an MPC Path Tracking controller(LQR in the backend) for a differential-drive robot tested in Gazebo simulation. Capstone project for Udacity C++ Nanodegree. Details on how to use the ROS package is mentioned here. You can also find below a brief description of the LQR controller and some information on how to tune it. This project was built and tested in Ubuntu 16.04 and ROS Kinetic + Gazebo7. 



## Path Tracking Control
Given a reference path, path tracking control is an algorithm to compute input commands for the robot to ensure it traverses the reference path as close as possible. Contrary to trajectory tracking, this algorithm doesn't account for the time it takes to traverse the path and only try to minimize the offset between the reference path and the actual traversed path. 

Given the current state and a "goal" state of the system, Model Predictive Control(MPC) is an control paradigm to compute input commands to drive the system to the goal state as below, 

-  A prior model of the system is used to numerically solve an open-loop optimization problem at each step in a look-ahead prediction time horizon. 
- Even though we have the control commands for all time steps in the current time horizon, only the first control command is executed.
- During the next time step, current state of the system is measured and the algorithm starts again, running till the system reaches and stays at the "goal" state.  

In this project, the optimization algorithm used in the backend for each time step is a Linear Quadratic Regulator. This project uses a neat closed form solution to this optimization problem(look at References). Although, the differential-drive model(our system) is non-linear, we approximate it as a linear system at each time step when the LQR is run. Pseudocode for the MPC and LQR can be found below. 



```
current_control_command func MPC(current_pose, reference_path)
    WHILE DISTANCE(current_pose, ENDOF(reference_path) < THRESHOLD) DO:
        array commands = OPTIMIZECURRENTHORIZON(current_pose, CURRENTHORIZON(reference_path))   
        var current_pose = GETCURRENTPOSE() 
    end WHILE
    return commands[0]
end func
```

```
commands func OPTIMIZECURRENTHORIZON(current_pose, current_horizon)
    var reference_input = {0}
    array commands
    FOR goal_pose in arr(END(current_horizon), START(current_horizon))
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


## Installation
Install dependancies 
##### Eigen

```bash
cd /home/workspace
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


##### source current workspace in required terminals
```bash
cd catkin_ws 
source devel/setup.bash
```

Record Path
```bash
roslaunch udacity_proj_pkg record_path.launch file_name:="/path/to/store/recorded_path.txt" 

example: 
roslaunch udacity_proj_pkg record_path.launch file_name:="/home/workspace/catkin_ws/udacity_proj/udacity_proj_pkg/path/recorded_path.txt" 
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

##### This command should open an Rviz GUI window showing the green path as the reference path(stored using the record path mode), red as the robot's tracked path and red is the predicted path in the current look ahead time horizon

Note: If this error pops up: 

```
ModuleNotFoundError: No module named 'rospkg'
```
please run 

```
pip install rospkg
```
## Expected Behavior and Rubrics(for evaluation by Udacity)

This application has two modes of operation - 1. Recording a path 2. Tracking the path. And instructions on how to run both the modes can be seen in "Usage". 

When recording a path, the location of the *.txt file storing the path is passed as an argument. A teleoperation node is used to control the robot and the path traced while manually teleoperating is stored in the *.txt file. During this operation, one can see the recorded path and current pose of the robot in the Rviz GUI.

When tracking the path, the location of *.txt file containing the recorded path is entererd in the params .yaml file, which also contains the parameters used to tune the algorithm. While tracking, the robot's state is queried from gazebo and therefore we have the complete access to the robot's true state. During this mode, one can see the reference path, tracked path and the predicted path in the current time horizon in Rviz GUI. This mode also leverages concurrency by running a parallel thread to compute the predicted path in the current timehorizon and publishes it. To synchronize this operation, a concurrent message queue is used. 


## CodeBase

#### Record path
- RecordPath.cpp
    - Main function for recording path mode. Gets the *.txt file's path. Instantiates the ROS node

- GazeoROS.cpp
    - GazeboROS class 
        - used for recording a path. Uses gazebo_ros objects to query for robot's current pose and uses that to store the traced path


#### Track path
- main.cpp
    - Main function for tracking path mode. Instantiates the ROS node and initiates the path tracking algorithm.

- PathTracker.cpp
    - PathTracker class(public PathTrackerROS, public LQR)
        - Complete encapsulated Path Tracking object that continuously runs the LQR algorithm for the current time horizon
        - Inherits LQR, PathTracker members.
    - PathTrackerROS class
        - Encapsulates ROS stuff related to path tracking. Publishes control command, current pose, and tracked, reference, and predicted paths. 

- LQR.cpp
    - LQR class (public ParseParam)
        - This class contains the method to get current control command and also runs a thread to compute predicted path based on the computed control commands for the current time horizon.
        - contains a message queue object used for synchronization.
        - Inherits from ParseParam
        - Gets all required parameters for the control algorithm from config/controller_params.yaml file using ParseParam's methods
    - MessageQueue class
        - Used to synchronize computation of predicted path and publishing of paths and control command.

- ParseParam.cpp
    - ParseParam class
        - Contains methods to convert ros_param strings to double and Eigen matrices as required by LQR
## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.
<!--
### To do
- Doxygen documentation(ROS doc)
- write test cases using example reference paths
- Iterative version of LQR
- Use dynamic model instead of kinetic
- Use non-linear solvers to incorporate constraints-->

## References
- [CSC2621 2019 course notes, Florian Shkurti ](http://www.cs.toronto.edu/~florian/courses/imitation_learning/lectures/Lecture2.pdf)
- [Optimal Control for Linear Dynamical Systems and Quadratic Cost, Pieter Abbeel](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf)
