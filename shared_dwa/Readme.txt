# shared_dwa package

This package is the algorithm of shared control with DWA (dynamic window approach).

The basic idea comes from the paper [The Shared Control Dynamic Window Approach for Non-Holonomic Semi-Autonomous Robots](https://ieeexplore.ieee.org/document/6840152)

## subscribed topics

- */odom*: the odometry of the agent, used for linear & angular velocity estimation
- */scan*: the lidar info, used for collision detection and obstacle avoidance
- */user/cmd_vel*: the standardized user input, denoted as desired linear & angular velocities

## published topics

- */shared_dwa/cmd_vel*: the final velocity command sent to the actuators



