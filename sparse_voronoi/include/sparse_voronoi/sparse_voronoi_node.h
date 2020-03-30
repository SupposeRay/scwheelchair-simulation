// ROS
#include <ros/ros.h>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// Messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <cstdio>
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <iso646.h>
#include <deque>
#include <numeric>
// Package
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>
#include <scat_libs/base_utils.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/tf_utils.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/obst_utils.h>
#include <sparse_voronoi/voronoi_algorithm.h>

namespace sparse_voronoi
{

class VoroNode
{
public:
  // Constructor
  VoroNode(ros::NodeHandle &node_handle);
  // Destructor
  virtual ~VoroNode();

private:
  //// Node methods
  bool readParameters();
  void laserObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects);
  void camObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects);
  void userinputCallback(const geometry_msgs::Twist::ConstPtr &msg_userinput);
  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_position);
  void targetCallback(const geometry_msgs::PoseArray::ConstPtr &msg_targets);
  void publishResults();
  // The main functionality from within voronoi_algorithm is run repeatedly, from within the timerCallback. 
  void timerCallback(const ros::TimerEvent &);

  // processObstacles() combines the obstacle messages received from the laserObjectCallback and camObjectcallback
  // and transfers them to the voronoi_algorithm object. 
  void processObstacles();

  //// Variables
  // Booleans for message reception
  bool pos_received_ = false;
  bool cmd_received_ = false;
  bool laser_obstacles_received_ = false;
  bool cam_obstacles_received_ = false;
  bool targets_received_ = true;
  bool node_active_ = true;
  bool interface_movebase_ = false;
  double x_agent_ = 0, y_agent_ = 0, theta_agent_ = 0;

  //// State containers
  scat_msgs::EnvObjectList cam_object_msg_, laser_object_msg_;
  geometry_msgs::PoseArray voro_targets_;
  geometry_msgs::Pose2D global_pose2D_;
  geometry_msgs::Twist cmd_vel_;
  nav_msgs::Path user_path_;
  std_msgs::Header cam_header_, laser_header_, base_header_;
  
  //// Parameters
  // Reference frame. All incoming obstacles must have this frame id. If it represents e.g. map, localization is needed.
  // All calcuations are done in this frame_id. If it represents a local frame_id (e.g. base_footprint), no localization is needed.
  std::string fixed_frame_id_ = "map", base_frame_id_ = "base_footprint";
  // Time parameters for regulating calculation frequency
  double pub_frequency_ = 10.0, calc_frequency_ = 10.0;
  // Time parameters for regulating calculation frequency
  ros::Time t_now_, t_prev_;  
  // prediction horizon
  int Np_ = 10;
  // time interval used for user path prediction
  double time_interval_ = 0.1;
  // range within which obstacles are considered
  double obst_range_ = 10.0;
  // threshold for activation based on joystick input
  double v_threshold_ = 0.1, w_threshold_ = 0.1;

  //// ROS objects
  ros::NodeHandle &node_handle_;

  ros::Subscriber laser_obstacle_subscriber_, cam_obstacle_subscriber_, 
  userinput_subscriber_, position_subscriber_, target_subscriber_;

  ros::Publisher obstacle_publisher_, vorolines_publisher_, delaunaylines_publisher_,
      voropath_publisher_, userpath_publisher_, fullpath_publisher_;
  ros::Publisher interppath_publisher_, goal_publisher_;

  ros::Timer timer_;

  // Voronoi algorithm object
  Voronoi_Algorithm voronoi_algorithm_;

  tf::TransformListener tf_listener_;
};

} // namespace sparse_voronoi
