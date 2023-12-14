# Shared Control Wheelchair Simulation

Maintained by LEI Zhen <zhen001@e.ntu.edu.sg>

Voronoi Planners & Gazebo Crowd Simulation contributed by BANG YI, https://github.com/bangyii

## Branch Descriptions
- `dynamic_dwa:` Contains modifications to shared_dwa which enables dynamic obstacle avoidance using constant velocity forward simulation for detected & tracked dynamic obstacles. Detection and tracking of dynamic obstacles is provided by a separate node, eg tysik's detector: https://github.com/tysik/obstacle_detector

-  `dynamic_dwa_spawn:` Contains first iteration of Gazebo Crowd Simulator which attempts to enable runtime spawn and de-spawn of actors/pedestrians in gazebo. Branch is mostly abandoned due to memory leak in Gazebo source, causing removed actors & models to continue using up memory. Therefore spawning and de-spawning eventually leads to gazebo crashing within a few minutes. Runtime spawning requires modification to gazebo source to remove micro-freezing during spawn.

- `dynamic_dwa_recycle:` Contains second iteration of Gazebo Crowd Simulator which replaces runtime spawning and de-spawn for recycling of pedestrians. When pedestrians are no longer used, they are put underground instead of being removed. Number of pedestrians in world are fixed throughout simulation.

- `dynamic_dwa_recycle_gamma_refactor:` WIP refactoring of gamma_simulator to decouple its logic/contents from ROS node src file.

## Simulation Initialization

1. Run Gazebo simulation environment with the museum scene and the robot

run in terminal: `roslaunch mbot_gazebo view_mbot_gazebo_museum.launch`

2. Run the Rviz visualization script

run in terminal: `roslaunch scwheelchair_launch visualization.launch`

3. Run the control_keyboard script (optional)

run in terminal: `roslaunch control_keyboard control_keyboard.launch`

4. Run the input_converter script to convert joystick input or keyboard input into velocity command

run in terminal: `roslaunch input_converter inputer_converter.launch`

5. Run the shared_dwa package to do the shared control

run in terminal: `roslaunch shared_dwa shared_dwa.launch`

### For the detailed parameter setting of each package, please check the readme and config files of each package
