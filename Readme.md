# Shared Control Wheelchair Simulation

Maintained by LEI Zhen <zhen001@e.ntu.edu.sg>

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
