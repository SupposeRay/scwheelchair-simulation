#include "scan_filter/scan_filter_node.h"

int main(int argc, char **argv)
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  
  ros::init(argc, argv, "scan_filter");
  ros::NodeHandle node_handle("~");

  scan_filter::ScanFilterNode ScanFilterNode(node_handle);

  ros::spin();

  return 0;
}