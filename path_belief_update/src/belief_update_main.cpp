#include "path_belief_update/belief_update_node.h"

int main(int argc, char **argv)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  ros::init(argc, argv, "belief_update");
  ros::NodeHandle node_handle("~");  

  belief_update::belief_updateNode belief_update(node_handle);

  ros::spin();

  return 0;
}