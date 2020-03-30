# shared_mpc

## Brief description
Performs gradient based MPC shared control, using IPOPT (Interior Point OPTimization) CPPAD (C++ Automatic Differentiation).
User input is provided via the linear and angular velocity of a geometry_msgs::Twist message. 
Obstacle information is supplied according to the standardized Object description scat_msgs::EnvObject. 
The node translates this obstacle information into a list of ellipses and circles, and tries to find a collision avoiding path, following user input.
All obstacle avoidance happens in the local frame and no global localization is required. 
A voronoi path estimator (see **sparse_voronoi**) has been included in the node, which can be used to have a safe first estimate for the optimization. 

The optimization is: 
Minimize: Kv*(velocity_user - velocity_mpc)^2 + Kh*(direction_user - direction_mpc)^2 + Ks*(v_t - v_t-1)^2
Subject to Collision constraints and Model Constraints. 
For more technical information see the 'docs' folder.

## Node description

### Parameters
- **node_active**: can be used to deactivate the node when in autonomous / manual control. 
- **debug**: can be used to print debugging info.
- **fixed_frame_id**: 'global' frame_id. 
- **base_frame_id**: 'local' frame_id. 
- **use_corner_constraints**: if "true", the algorithm additionally places circles at each corner of an ellipse, increasing the safety margin when going around corners. However this adds to computational cost. 
- **use_voronoi**: if "true", uses the voronoi optimal path estimate to initialize the optimization algorithm. 
- **use_adaptive**: if "true", listens for **/user/performance** parameters and adapts costfunction parameters accordingly. 
- **obst_dist_threshold**: preferred distance from obstacles. This determines the distance to walls (thickness of ellipse) and circles. 
- **calc_frequency**: rate at which mpc is performed. 
- **obst_frequency**: rate at which new obstacle information is received and processed. 
- **obst_range**: Obstacles farther away than this are filtered out of the calculation. Make sure that this value is higher than the max path length (=prediction_horizon * sampling_time * max_velocity). 
- **prediction_horizon**: number of path samples predicted and optimized. 
- **sampling_time**: time between path samples. 
- **Kv:** cost function parameter for obedience to user linear velocity. 
- **Kh:** cost function parameter for obedience to user direction. 
- **Kw:** cost function parameter for obedience to user angular velocity (not commonly used). 
- **Ks:** cost function parameter for smoothness. 
- **Keq:** equality constraint multiplier (for rescaling adherence to dynamic model during optimization). 
- **Kineq:** inequality constraint multiplier (for rescaling obstacle avoidance during optimization). 

Parameters for adaptive shared control:
If *use_adaptive** = true, the cost function parameters are re-calculated every time a user performance message is received, according to:
K = Kx0 + Kxr * (1-perf_x) where perf_x is the user performance in a given direction scaled from 0 to 1.

- **Kv0:** lowest possible value for Kv. 
- **Kh0:** lowest possible value for Kh. 
- **Kw0:** lowest possible value for Kw. 
- **Kv0:** Additional range for Kv. 
- **Kh0:** Additional range for Kh. 
- **Kw0:** Additional range for Kw. 

Robot specific parameters:
- **v_max** [m/s]: max linear velocity
- **v_min** [m/s]: min linear velocity
- **v_max_neg** [m/s]: max negative linear velocity
- **v_threshold** [m/s]: threshold linear velocity. commands below are ignored. 
- **w_max** [rad/s]: max angular velocity
- **w_min** [rad/s]: min angular velocity. This should normally be 2*v_min/base_width
- **w_threshold** [rad/s]: threshold angular velocity. commands below are ignored. 
- **acc_lim_x** [m/s^2]:  max linear acceleration
- **dec_lim_x** [m/s^2]:  max linear deceleration
- **acc_lim_theta** [rad/s^2]:  max angular acceleration
- **dec_lim_theta** [rad/s^2]: max angular deceleration
- **base_width** [m]: width between two wheels of differential drive robot
- **footprint** [m,..]: footprint loaded as list of points, i.e. [x1, y1, x2, y2 ..]
- **robot_radius** [m]: if radius is used instead of footprint for collision avoidance

### Subscribers

- **/user/cmd_vel** (geometry_msgs/Twist):
User input

- **/user/performance** (std_msgs/Float32MultiArray):
Consists of entries of the [cov_vx, cov_vy, cov_vxy].
Used to make the shared control adaptive to user skill (mainly tremor).
This functionality can be enable/disable via parameter settings. 

- **/line_segments** (scat_msgs/EnvObjectList):
Obstacle descriptions

- **/amcl_pose** [geometry_msgs/PoseWithCovarianceStamped]:
global position

### Publishers

- **/control/cmd_vel** (geometry_msgs/Twist)

The following publishers publish visualization messages, used for visualization in rviz. 
- **/shared_mpc/mpc_path**: Path calculated by MPC
- **/shared_mpc/user_path**: Original path predicted from user input. 
- **/shared_mpc/voro_path**: Path from the Voronoi algorithm. 
- **/shared_mpc/voronoi_lines**
- **/shared_mpc/delaunay_lines**

### Services

- **/shared_mpc/activation_service**
Service used to enable/disable this node. 

## Dependencies
Shared MPC depends on IPOPT and CPPAD.
Please follow the instructions in docs/InstallingIpopt/CarND-MPC-Project/install_Ipopt_CppAD.md
