# trajector_generator

## Brief description:
Calculates a trajectory based on userinput.
The trajectory avoids obstacles which are in the costmap.

## Node description

### Subscribers:
/user/cmd_vel [geometry_msgs/Twist]
/odom [nav_msgs/Odometry]
/amcl_pose [geometry_msgs/PoseWithCovarianceStamped]
/move_base/global_costmap/costmap [nav_msgs/OccupancyGrid]

### Publishers:
/trajectory_generator/trajectory [nav_msgs/Path]
/trajectory_generator/goal [geometry_msgs/PoseStamped]

### Services:

/trajectory_generator/activation_service
Service used to enable/disable this node. 
