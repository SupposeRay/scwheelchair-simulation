#include "joystick_goal_node.h"

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "joystick_goal_node");
  ros::NodeHandle node_handle("~");
   
  joystick_goal_node::JoystickGoalNode JoystickGoalNode(node_handle); 

  ros::spin();
    
  return 0;
}
