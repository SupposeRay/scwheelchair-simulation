# The Belief-based Shared Control

## subscribed topics

- */odom*: the odometry of the agent
- */input_converter/cmd_vel*: the user's command velocity
- */all_paths*: all the path candidates from different homotopic classes

## published topics

- */most_likely_goal*: the predicted most likely goal for shared_dwa

## Simulation Initialization

1. Run Gazebo simulation environment

run in terminal: `roslaunch mbot_gazebo view_mbot_gazebo_museum.launch`

2. Run the Rviz visualization script

run in terminal: `roslaunch mbot_navigation navigation.launch`

3. Run the scat_move_base package

run in terminal: `roslaunch scat_move_base move_base.launch`

4. Run the belief update package

run in terminal: `roslaunch path_belief_update belief_update.launch`

5. Run the user input package (joystick or keyboard)

Make sure the user's input is a `geometry_msgs/Twist` message and is published to `/joy_vel`


### Please check the config file for more parameters setting