#include "shared_dwa/shared_DWA_node.h"

int main(int argc, char **argv)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  ros::init(argc, argv, "shared_dwa");
  ros::NodeHandle node_handle("~");  

  shared_DWA::shared_DWANode Shared_DWA(node_handle);

  // ros::spin();

  return 0;
}