#include "joystick_filter/joystick_filter_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_filter_node");
  ros::NodeHandle node_handle("~");

  joystick_filter_node::FilterNode FilterNode(node_handle); 

  ros::spin();
    
  return 0;
}
