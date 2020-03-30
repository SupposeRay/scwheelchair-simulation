// ROS
#include <ros/ros.h>
// Messages
#include <scat_msgs/EnvObject.h>
#include <scat_msgs/EnvObjectList.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// C++
#include <stdlib.h>
#include <math.h>
// Package
#include <scat_libs/rosmsg.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/nav_utils.h>

namespace test_obstacle_publisher {

class ObstaclePublisher
{
 public:
  
  ObstaclePublisher(ros::NodeHandle& node_handle);
  virtual ~ObstaclePublisher();

 private:
  // Node Methods
  bool readParameters();
  void timerCallback(const ros::TimerEvent&);  

  // Node parameters
  float publish_frequency_ = 10;
  scat_msgs::EnvObjectList object_msg_list_;
  visualization_msgs::MarkerArray object_marker_list_;
  std::string frame_id_ = "map";

  // ROS objects
  ros::NodeHandle& node_handle_;
  ros::Publisher obst_marker_publisher_, obst_msg_publisher_;
  ros::Timer timer_; 
};

} /* namespace */
