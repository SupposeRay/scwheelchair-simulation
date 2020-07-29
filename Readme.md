# Scatwheel Drivers Repository

## Introduction
This Repository contains the packages designed for Shared Control of a differential drive robot, purposed for the Shared Control All Terrain wheelchair project (SCATwheel).
Additional information on each package can be found in the Readme's and docs folders inside the package folders.

## Remote connection

Below is a short manual for how to connect from a remote PC (connected to the same Wifi Network) to the robot PC's. 
This is done using ssh and the ROS network connection. Topics are published from either PC to the network IP address, and therefore ROS Nodes on the Robot PC can easily talk to ROS Nodes on the Remote PC.
There will be some delay however due to the network connection. Therefore all safety critical nodes have to be run on the Robot PC itself, and the remote connection is mostly used for startup, visualization, data collection etcetera. 

For more information, visit:

- http://wiki.ros.org/ROS/NetworkSetup
- http://wiki.ros.org/ROS/Tutorials/MultipleMachines

Current network settings:

- Network: RoboTenda
- Wifi Password: scatwheel
- SCATwheel (fixed) IP address (= **robot_network_ip**): 192.168.0.101
- SCATwheel PC name ( = **robot_pc_name**): scatwheel-nuc8i5beh
- SCATwheel user name ( = **robot_username**): scatwheel
- Turtlebot (fixed) IP address (= **robot_network_ip**): 192.168.0.100
- Laptop IP address example (= **remote_network_ip**): 192.168.0.102

1: Network setup

These commands can be used to install and setup ssh, which is required from making the remote connection on Ubuntu. 
The Robot PC will usually be the Host PC for ssh connection, while the Remote PC will be the Client PC.

- Install SSH on Client PC:

    - $ sudo apt update
    - $ sudo apt install openssh-server

- Check if installation has worked:

    - $ sudo systemctl status ssh

- log in to ssh:

    - $ ssh username@IP-address

- Change password shh:

    - $ sudo -s
    - $ passwd
    - $ exit

2: ROS connection

Once you have setup the network, take the following steps. 

- Check IP address on Robot PC and Remote PC 

    - $ hostname -I
    - returns: IP-address-host
    - Should be: 192.168.0.101 at SCATwheel PC.

- Add IP address of robot to the list of known hosts on your Ubuntu PC:

    - $ sudo gedit /etc/hosts
    - Write "**robot_network_ip** **robot_pc_name**", for example: "192.168.0.101	scatwheel-nuc8i5beh", below the other hosts

- Add IP address of your remote PC to the list of known hosts on the robot PC:

    - $ sudo gedit /etc/hosts
    - Write "**remote_network_ip**	 **remote_pc_name**"  below the other hosts
  
- Change ROS settings on Remote PC:

    - $ gedit ~/.bashrc
    - Paste at the end:
    - $ ROS_IP=**remote_network_IP**				            make sure ROS on Remote PC listens to its IP port
    - $ ROS_MASTER_URI=http://**robot_network_IP**:11311		make sure ROS on Remote PC knows where to locate the ROS master, located at the IP address of the Robot PC.

- Open new terminal on Remote PC (Client) and make the connection to Robot PC (Host):

    - $ ssh **robot_username**@**robot_network_IP**			    this logs into the robot PC. Example: $ ssh scatwheel@192.168.0.101
    - $ roscore						                        start roscore on robot pc.
  
- Open new terminal on Remote PC and check if ROS is connected to the Robot PC:

    - $ rostopic list


## Robot startup

Launch files and custom ROS messages are  placed in separate packages as per good practice. 
There are a few options to launch the robot software. These options are brought together in the **scat_launch** package:

Usage: 
** roslaunch scat_launch ... **

- **full_robot.launch**: 
Brings up the full repository, including autonomous navigation, shared control and manual mode, with a master node able to switch between the two. 
The system will always start in autonomous navigation mode. Giving some input with the joystick will automatically switch to shared control mode.
Sending some goal (at /move_base_simple/goal) will automatically switch back to autonomous navigation mode. 

- **autonomous_mode.launch**:
Brings up the robot with autonomous mode only. 

- **sc_mode.launch**: 
Brings up the robot with Shared Control mode only. Optionally with with global localization, which allows shared controd modes using move_base local planner. 
Switching between different modes can be done in the launch file args (before launching). 

- **manual_mode.launch**: 
Brings up the robot in Manual mode only. This includes the joystick filter and velocity safety filter (with collision and acceleration/velocity check).

- **slam_mode.launch**:
Launches the robot in SLAM mode (using gmapping).

The required topics from hardware/simulation for all launch files are:

- **/user/joy** [std_msgs/Float32MultiArray]:     user joystick input
- **/encoders/odom** [nav_msgs/Odometry]:     velocity and position estimation based on encoder data alone
- **/scan** [sensor_msgs/LaserScan]:                     laser scan
- **/imu** [sensor_msgs/Imu]:                imu

Main parameters (launch arg's) you can set inside the launch files or from command line:

- **sim**: if "false", launches the robot hardware_drivers. 

- **sc_type**: Selects shared control type.
  
    - **mpc**: Uses gradient-based Model Predictive Control to generate and follow the trajectory. See **shared_mpc**
    - **dwa**: Uses sampling-based Model Predictive Control to generate and follow the trajectory. See **shared_dwa**
    - **traj**: Uses the costmap to generate and move_base follow the trajectory. See **trajectory_generator**
    - **goal**: Uses the costmap to plot a goal and move_base follow the trajectory. See **joystick_goal**

- **map**: select map from the scat_launch/maps/ folder. e.g. rris.yaml or maze.yaml or some new custom map. 

- **joy_topic**: Set joy topic. Commonly used:
  
    - "user": selects joystick on wheelchair (joystick data sent by MCU)
    - "remote": selects joystick separately connected to remote PC< from Arduino. 
  
- **debug**: if true, prints a lot of debugging info. 


## Other launch files
The following launch files are launched from inside the core launch files. 
Therefore during normal use / testing you don't need to use / modify these. 
However for development / debugging they can also be launched separately. 

- **hardware_drivers.launch**:
Launches all the hardware drivers and sensor fusion drivers.

- **localization.launch**:
Launches the localization module. 

- **hardware_drivers.launch**:
Launches the shared control module. 

- **laser_obstacles.launch**:
Launches the laser obstacle extraction module. 

- **visualization.launch**:
Because the repository has been designed for remote initialization and debugging through ssh, no RViZ node has been included in previous launch files. 
This launch file can be used to launch an RViZ node with some presets. 

## Packages:
Overview of packages included in this repository:

### General packages
- scat_launch:
main package with launch files and configuration files.

- scat_master:
Master node which can switch between autonomous, shared and manual mode. 

- scat_libs:
Contains commonly used functions, handy for easy coding and readible nodes. 

- scat_msgs:
Contains all custom messages. 

### Hardware packages
- serial_communication:
Driver for MCU. Communication node which connects to the MCU, controlling the actuators and retrieving data from IMU, Encoders and Joystick. Also provides a simple Odom calculation based on Encoder velocities. 

- robot_tf:
Contains two nodes:
	tf_broadcaster: Publishes required robot transforms.
	odom_publisher: publishes odometry message based on laser_scan_matcher data and expected odomotry for cmd_vel (for calibration purpose).

- rplidar_ros:
Driver for LaserScanner.

- scan_filter:
Filter for LaserScanner.

- scan_merger:
Merges two scans to a single output scan. 

- laser_scan_matcher:
Compares two subsequent laser scans to measure the difference and estimate odometry. 

- robot_pose_ekf:
Uses an EKF to estimate odometry. 

- joystick_filter:
Alternative to control_joystick, which filters the joystick signal with a moving average or Kalman filter. 

- vel_safety_filter:
All velocity commands are checked by this node before being sent to the robot. 

### Shared control packages
- shared_mpc:
Gradient-based Shared Model Predictive Control

- shared_dwa:
Sampling-based Shared Model Predictive Control

- global_planner_plugin:
Plugin which interfaces between some trajectory published to /global_planner_plugin/trajectory and the ROS move-base local planner.

- joystick_goal:
Directly publishes a goal to to ROS move-base global planner, based on (filtered) joystick input. 

- sparse_voronoi:
Calculates voronoi diagrams and estimates user-intended trajectory. 

- trajectory_generator:
Publish trajectory based on user input and costmap. 

### Obstacle detection packages
- laserline:
Laser based line extraction

- laser_linear_extraction:
Laser based line extraction

- obstacle_detector:
Extracts line and circle obstacles from laser data, tracks circle obstacles.

### Packages for testing, calibration and debugging
- test_cmd_publisher:
Automatically publish a series of velocity control commands. Used for sensor calibration. 

- test_obstacle_publisher:
Used to publish fake obstacles of various types according to the scat_msgs::EnvObject format. 

- sensor_calibration:
Automatically calibrates IMU and Encoders. 

### External dependencies
ROS:

- ROS Kinetic
- move_base
- gmapping


non-ROS:

- ipopt + CPPAD (shared_mpc)
- openCV (trajectory_generator)
- Eigen3 (laser_line_extraction)


### Simulation Startup
1. Run Gazebo simulation environment with the museum scene and the robot
 run in terminal: roslaunch mbot_gazebo view_mbot_gazebo_museum.launch
2. Run the Rviz visualization script
 roslaunch scat_launch visualization.launch 
3. Run the control_keyboard script (optional)
 roslaunch control_keyboard control_keyboard.launch
4. Run the shared_dwa_lz package to do the shared control
 roslaunch shared_dwa_lz shared_dwa_lz.launch
 
