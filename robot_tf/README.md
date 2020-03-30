# robot_tf

## Brief description
this package can be used to publish transform and odometry data

## tf_publisher
This node is used to broadcast the essential TF frames. 
Provided transforms:
base_footprint > base_link
base_link > laser_left
base_link > laser_right
base_link > imu

## odom_publisher
Additionally it contains this helper node which publishes odometry data based on velocity and/or position input.
The odometry publisher is for testing only, not used in the final implementation.

odom_publisher subscriptions:
/cmd_vel [geometry_msgs/Twist]
/Pose2D [geometry_msgs/Pose2D]
odom_publisher publishers:
/odom_scan [nav_msgs/Odometry]
odometry based on pose2d data (coming from scan)
/vel_scan [geometry_msgs/Twist]
velocity based on pose2d data (coming from scan)
/odom_cmd [nav_msgs/Odometry]
odometry based on cmd_vel data

