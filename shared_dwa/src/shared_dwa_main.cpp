#include "shared_dwa/shared_dwa_node.h"

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "shared_dwa");
  ros::NodeHandle node_handle("~");

  shared_dwa::DWANode DWANode(node_handle);

  ros::spin();

  return 0;
}