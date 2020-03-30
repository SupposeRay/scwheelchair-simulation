# joystick_goal

## Brief description

Takes in joystick input and publishes a goal in the map in the direction where the joystick is pointing.
The algorithm additionally uses the costmap to place the goal in a safe location (avoiding obstacles in direct line of sight towards the goal).

## Node description

### Subscribers
/user/input
/amcl_pose
/move_base/global_costmap/costmap

### Publishers
/user/goal (PoseStamped):
	Goal placed at safe location.
/user/initial_goal (PoseStamped):
	Goal position calculated directly from user input.

