# Velocity filter

## Brief description
Takes in a velocity pair (Twist) and outputs a new velocity pair which takes into account:

- Velocity limits
- Acceleration limits
- Collisions.

The velocity and acceleration are commonly supplied in a seperate file. 
To accomodate for different modes of the robot, the velocity commands can be supplied to two inputs:
/control/cmd_vel (when in autonomous / shared control mode) or /user/cmd_vel (when in manual mode). 
However the node needs to know in which mode the system is, i.e. to which it should list. 
This mode can be selected via a the service. See below for more information. 

## Node description

### Parameters

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
- **vel_type**: 
    - "odom": listens for odometry type message and uses this for setting the dynamic window. 
    - "twist": listens for twist type message and uses this for setting the dynamic window. 	
- **use_vel_filter**: if "true" checks for velocity limits.
- **use_acc_filter**: if "true" checks for acceleration limits.
- **use_coll_filter**: if "true" checks for collisions.
- **footprint_inflation**: footprint multiplication factor. 0.2 = 20% inflation
- **vel_inflation**: additional inflation factor dependent on velocity. Increase if getting stuck alongside the wall. Full inflation (e.g. 20% for factor 0.2) is added when v = v_max.
- **obst_inflation**: !Only used for radius collision method! make sure this is larger than than vel_safety_filter obstacle inflation
- **angle_res** [rad]: Angular resolution when check if indicated velocity commands are towards or away from obstacles. 
- **obst_range** [m]: Obstacles farther away than this are filtered out of the calculation. Make sure that this value is higher than v_max * (time_interval * path_samples + 1/pos_update_freq) + obst_inflation
- **sampling_time** [s]: Used for deceleration when collision is detected, checking a new velocity: v_new = velocity - deceleration * sampling_time
- **time_out** [s]: If no new velocity commands (cmd_vel) have been received for this duration, automatically resets velocity command to 0 to avoid continuing after control is lost. 

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

### Subscribers: 

- **/control/cmd_vel** [geometry_msgs/Twist]: 
command velocity input when in shared control of autonomous navigation mode. 

- **/user/cmd_vel** [geometry_msgs/Twist]
Command velocity input when in manual mode. 

Obstacle information can be supplied via:

Either of the following can be used for obstacle data:

- **/scan** (sensor_msgs/LaserScan)
- **/obstacles** (scat_msgs/EnvObjectList)
- **/pointcloud** (sensor_msgs/PointCloud2)

Either of the following can be used for current velocity data:

- **/odom** (nav_msgs/Odometry)
- **/vel** (geometry_msgs/Twist)

### Publishers:

- **/control/cmd_vel** (geometry_msgs/Twist): velocity command to be sent to robot. 

### Services:

- **/vel_safety_filter/mode_service**:
service used to set the mode (autonomous/shared/manual) of the wheelchair. 
This determines which control topic (/control/cmd_vel or /user/cmd_vel) is used. 

