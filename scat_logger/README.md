Logger node for recording a reference path by entering reference path mode, and then driving robot around.

User path mode will record the user's path and joystick inputs.

Robot's position is computed by transforming base_link to map frame, so any drifts in odometry or jumps in map -> odom frame will affect recorded data.

Several parameters can be set in the launch file.

Read the instructions on console when running the node properly, there is only a few input checks, so if the node does not work as expected, double check the params and that you followed the instructions clearly.