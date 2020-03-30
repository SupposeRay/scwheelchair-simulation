#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/SetBool.h>

// C++
#include <stdlib.h>
#include <math.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace joystick_goal_node
{

class JoystickGoalNode
{
public:
  // Constructor
  JoystickGoalNode(ros::NodeHandle &nodeHandle);
  // Destructor
  virtual ~JoystickGoalNode();

private:
  // Node methods
  bool readParameters();
  void joyCallback(const std_msgs::Float32MultiArray &msg_userinput);
  void mapCallback(const nav_msgs::OccupancyGrid &occupancyGrid);
  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped &msg_position);
  void odomCallback(const nav_msgs::Odometry &msg_position);
  void costmapCallback(const nav_msgs::OccupancyGrid &msg_costmap);
  void publishResults();
  bool serviceCallback(std_srvs::SetBoolRequest &request,
                       std_srvs::SetBoolResponse &response);

  // Algorithm methods
  geometry_msgs::Point goalLineOfSight(double x_goal, double y_goal, double x_agent, double y_agent);
  void generateCVMap(nav_msgs::OccupancyGrid map);
  bool checkForCollision(int index_row, int index_col);
  bool checkMapBoundary(int index_row, int index_col);

  //// State Containers ////
  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::PoseStamped goal_, initial_goal_;
  geometry_msgs::PoseStamped agent_pos_;
  nav_msgs::Path path_;
  nav_msgs::OccupancyGrid map_;
  ros::Time t_, t_prev_;
  cv::Mat map_cv_;

  //// Variables ////
  // map size
  int map_cols_, map_rows_;
  // map resolution
  double map_resolution_;
  // Agent position
  double x_agent_, y_agent_, theta_agent_;
  // origin of costmap with respect to current map frame
  double origin_x_, origin_y_, origin_z_, origin_theta_;
  // Resolution path
  double path_resolution_;
  // Some booleans for message reception
  bool joy_received_ = false;
  bool position_received_ = false;
  bool map_received_ = false;

  //// Node parameters ////
  bool node_active_ = false;
  // Max goal distance
  double goal_dist_max_ = 5;
  // Publish frequency
  double publish_frequency_ = 4;
  // Threshold for collision
  int coll_threshold_ = 20;
  // joy treshold
  double joy_threshold_ = 0.3;

  //// ROS objects ////
  ros::NodeHandle &node_handle_;
  // Subscribers
  ros::Subscriber joy_subscriber_, position_subscriber_, odom_subscriber_, map_subscriber_;
  // Publishers
  ros::Publisher goal_publisher_, initial_goal_publisher_, path_publisher_;
  // Activation service
  ros::ServiceServer activation_service_;
};

} // namespace joystick_goal_node
