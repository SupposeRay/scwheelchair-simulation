# shared_dwa

## Brief Description

This package contains a local planning algorithm based on the "Shared Dynamic Window Approach"
It can also be described as "velocity-based DWA". 
It uses the direct user velocity commands as input, and generates the optimal velocity commands as output.
Obstacle avoidance can be achieved using both costmap and scan. 
Using the parameter settings it can be selected whether to use scan, costmap or both. 
Using the parameter settings it can also be selected whether to use Odometry or a simple velocity Twist message as source of current robot velocity. 

## Node description

### Parameters
- **node_active**: can be used to deactivate the node when in autonomous / manual control. 
- **debug**: can be used to print debugging info.
- **fixed_frame_id**: 'global' frame_id. 
- **base_frame_id**: 'local' frame_id. 

- **collision_method**: Method used to check for collision.
    - "radius": just checks the distance to obstacles and reports collision if the distance is smaller than obst_inflation.
    - "footprint": checks whether the provided footprint is in collision with known obstacles, using obst_inflation, footprint_inflation and vel_inflation. footprint is more accurate. 

- **obst_type**: options: "map", "scan", "cloud" or "obstmsg" 
    - "scan" can be used with either footprint or radius mode. scan is generally the fastest. 
    - "map" uses the occupancyGrid published at "/move_base/local_costmap/costmap" to avoid map based obstacles. 'collision_method' is automatically changed to "radius" when using "map" method. "map" is generally the slowest. 
    - "cloud" uses a 3d pointcloud, e.g. coming from rgbd image or Laserscan in pointcloud format. 
    - "obstmsg" uses the obstacle message description scat_msgs::EnvObjectList.

- **use_ref_trajectory**: if "true", uses a reference trajectory in the cost function, slightly guiding the user along. This reference trajectory could be coming from the **sparse_voronoi** algorithm but not necessarily. 
- **use_coll_margin**: if "true", uses a reference trajectory in the cost function, slightly guiding the user along. This reference trajectory could be coming from the **sparse_voronoi** algorithm but not necessarily. 
- **use_adaptive**: if "true", listens for **/user/performance** parameters and adapts costfunction parameters accordingly. 
- **vel_type**: 
    - "odom": listens for odometry type message and uses this for setting the dynamic window. 
    - "twist": listens for twist type message and uses this for setting the dynamic window. 
- **footprint_inflation**: footprint multiplication factor. 0.2 = 20% inflation
- **vel_inflation**: additional inflation factor dependent on velocity. Increase if getting stuck alongside the wall. Full inflation (e.g. 20% for factor 0.2) is added when v = v_max.
- **obst_inflation**: !Only used for radius collision method! make sure this is larger than than vel_safety_filter obstacle inflation
- **obst_range** [m]: Obstacles farther away than this are filtered out of the calculation. Make sure that this value is higher than v_max * (time_interval * path_samples + 1/pos_update_freq) + obst_inflation
- **occupancy_threshold**: when costmap is used for obstacle avoidance, this parameter determines when a field is considered to be an obstacle. 
- **pos_update_freq**: when costmap is used for obstacle avoidance, this parameter determines how often current position is updated. 
- **prediction_horizon**: number of path samples predicted and optimized. 
- **sampling_time**: time between path samples. 
- **samples_v**: number of samples in linear velocity
- **samples_w**: number of samples in angularity
- **obst_dist_threshold**: preferred distance from obstacles. This determines the distance to walls (thickness of ellipse) and circles. 
- **calc_frequency**: rate at which mpc is performed. 
- **obst_frequency**: rate at which new obstacle information is received and processed. 

- **Kv:** cost function parameter for obedience to user linear velocity. 
- **Kh:** cost function parameter for obedience to user direction. 
- **Kw:** cost function parameter for obedience to user angular velocity (not commonly used). 
- **Ks:** cost function parameter for smoothness. 
- **Kc:** 'collision margin' parameter. Can be added to increase the cost of comming close to obstacles. Only used if **use_coll_margin** = true. (not necessary / commonly used). Slows down computation time. 
- **Kref:** cost function parameter for obedience to reference trajectory. Only used if **use_ref_trajectory** = true. (not necessary / commonly used). 

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
user input.

- **/user/performance** (std_msgs/Float32MultiArray):
Consists of entries of the [cov_vx, cov_vy, cov_vxy].
Used to make the shared control adaptive to user skill (mainly tremor).
This functionality can be enable/disable via parameter settings. 

Either of the following can be used for obstacle data:
- **/scan** (sensor_msgs/LaserScan)
- **/move_base/local_costmap/costmap** (nav_msgs/OccupancyGrid)
- **/obstacles** (scat_msgs/EnvObjectList)
- **/pointcloud** (sensor_msgs/PointCloud2)

Either of the following can be used for current velocity data:
- **/odom** (nav_msgs/Odometry)
- **/vel** (geometry_msgs/Twist)

### Publishers

- **/control/cmd_vel** (geometry_msgs/Twist): velocity command to be sent to robot. 

- **/dwa_results**: (std_msgs/Float32MultiArray): 
Containing the following parameters, useful for debugging.

    - cost
	- clearance 
	- cost_heading
	- cost_velocity
	- cost_smoothness
	- cost_safety
	- av_path_length
	- av_danger
	- fraction_of_safe_paths

The following publishers publish visualization messages, used for visualization in rviz. 
- **/shared_dwa/dynamic_window**
- **/shared_dwa/dwa_path**
- **/shared_dwa/user_path**
- **/shared_dwa/dwa_shape**
- **/shared_dwa/user_shape**

### Services

- **/shared_dwa/activation_service**: 
Service used to enable/disable this node. 
