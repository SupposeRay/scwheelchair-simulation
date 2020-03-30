#include "test_obstacle_publisher/test_obstacle_publisher_node.h"

int main(int argc, char** argv)
{
  //! When debugging, set logger_level to Debug by changing Info to Debug. 
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  ros::init(argc, argv, "test_obstacle_publisher_node");
  ros::NodeHandle node_handle("~");
   
  test_obstacle_publisher::ObstaclePublisher ObstaclePublisher(node_handle); 

  ros::spin();
    
  return 0;
}
