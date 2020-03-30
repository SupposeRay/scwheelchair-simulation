#include "joystick_goal_node.h"

namespace joystick_goal_node
{

JoystickGoalNode::JoystickGoalNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
  ROS_DEBUG("Launching Constructor");

  if (!readParameters())
  {
    ROS_ERROR("Could not load parameters.");
    ros::requestShutdown();
  }

  // Subscribers & Publishers
  joy_subscriber_ = node_handle_.subscribe("/user/joy", 1, &JoystickGoalNode::joyCallback, this);
  position_subscriber_ = node_handle_.subscribe("/amcl_pose", 1, &JoystickGoalNode::positionCallback, this);
  // odom_subscriber_ = node_handle_.subscribe("/odom", 1, &JoystickGoalNode::odomCallback, this);
  map_subscriber_ = node_handle_.subscribe("/map", 1, &JoystickGoalNode::mapCallback, this);
  goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/user/goal", 1);
  initial_goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/user/initial_goal", 1);
  path_publisher_ = node_handle_.advertise<nav_msgs::Path>("/user/direct_path", 1);
  activation_service_ = node_handle_.advertiseService("activation_service", &JoystickGoalNode::serviceCallback, this);

  ROS_DEBUG("waiting for map");
  if (node_active_)
  {
    while (!map_received_)
    {
      ROS_DEBUG("Waiting for map");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
    ROS_DEBUG("map received");
  }
  ROS_DEBUG("Joystick Goal Node started succesfully");
}

JoystickGoalNode::~JoystickGoalNode() {}

bool JoystickGoalNode::readParameters()
{
  bool debug = false;
  if (!node_handle_.getParam("debug", debug))
    ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);    
  if (!node_handle_.getParam("node_active", node_active_))
    ROS_WARN_STREAM("Parameter node_active not set for joystick_goal. Using default setting: " << node_active_);
  if (!node_handle_.getParam("publish_frequency", publish_frequency_))
    ROS_WARN_STREAM("Parameter publish_frequency not set for joystick_goal. Using default setting: " << publish_frequency_);
  if (!node_handle_.getParam("goal_dist_max", goal_dist_max_))
    ROS_WARN_STREAM("Parameter goal_dist_max not set for joystick_goal. Using default setting: " << goal_dist_max_);
  if (!node_handle_.getParam("joy_threshold", joy_threshold_))
    ROS_WARN_STREAM("Parameter joy_threshold not set for joystick_goal. Using default setting: " << joy_threshold_);
  if (!node_handle_.getParam("coll_threshold", coll_threshold_))
    ROS_WARN_STREAM("Parameter coll_threshold not set for joystick_goal. Using default setting: " << coll_threshold_);


  // When debugging, set logger_level to Debug by changing Info to Debug.
  if (debug)
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  ROS_DEBUG("Parameters read.");
  return true;
}

void JoystickGoalNode::joyCallback(const std_msgs::Float32MultiArray &msg_userinput)
{
  if (node_active_)
  {
    // ROS_INFO_STREAM("received joy");
    joy_received_ = true;
    double joy_magnitude = sqrt(pow(msg_userinput.data[0], 2) + pow(msg_userinput.data[1], 2));
    // Every period, determined by min_dt, a new goal position will be calculated.
    // Update time measurement
    t_ = ros::Time::now();
    ros::Duration dt = t_ - t_prev_;
    if (dt.toSec() > 1/publish_frequency_)
    {
      if (map_received_)
      {
        t_prev_ = t_;
        goal_.header.frame_id = "map";
        goal_.header.stamp = ros::Time::now();

        double x_goal, y_goal, theta_goal;
        if (joy_magnitude > joy_threshold_)
        {
          // Use standard 2D rotation matrix to translate userinput to agent frame of reference
          x_goal = x_agent_ + goal_dist_max_ * (msg_userinput.data[0] * cos(theta_agent_) - msg_userinput.data[1] * sin(theta_agent_));
          y_goal = y_agent_ + goal_dist_max_ * (msg_userinput.data[0] * sin(theta_agent_) + msg_userinput.data[1] * cos(theta_agent_));
          theta_goal = theta_agent_ + atan2(msg_userinput.data[1], msg_userinput.data[0]); // angle of goal position w.r.t map origin
          goal_.pose.position = goalLineOfSight(x_goal, y_goal, x_agent_, y_agent_);
        }
        else
        {
          x_goal = x_agent_;
          y_goal = y_agent_;
          theta_goal = theta_agent_;
          goal_.pose.position.x = x_goal;
          goal_.pose.position.y = y_goal;
        }

        goal_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_goal);

        initial_goal_.header = goal_.header;
        initial_goal_.pose.position.x = x_goal;
        initial_goal_.pose.position.y = y_goal;
        initial_goal_.pose.orientation = goal_.pose.orientation;

        publishResults();
      }
      else
      {
        ROS_WARN_STREAM("joystick goal node waiting for map");
      }
    }
  }
}

void JoystickGoalNode::positionCallback(const geometry_msgs::PoseWithCovarianceStamped &msg_position)
{
  x_agent_ = msg_position.pose.pose.position.x;
  y_agent_ = msg_position.pose.pose.position.y;
  theta_agent_ = tf::getYaw(msg_position.pose.pose.orientation);
  position_received_ = true;
}

void JoystickGoalNode::mapCallback(const nav_msgs::OccupancyGrid &msg_map)
{
  map_ = msg_map;
  map_cols_ = map_.info.width;
  map_rows_ = map_.info.height;
  map_resolution_ = map_.info.resolution;
  origin_x_ = map_.info.origin.position.x;
  origin_y_ = map_.info.origin.position.y;
  origin_z_ = map_.info.origin.position.z;
  origin_theta_ = tf::getYaw(map_.info.origin.orientation);
  // Setting path resolution depending on map resolution.
  // Assuming that a block of 3 cells is checked for collision every time, no collision will be missed.
  path_resolution_ = 6 * map_resolution_;
  generateCVMap(map_);
  map_received_ = true;
}

void JoystickGoalNode::publishResults()
{
  initial_goal_publisher_.publish(initial_goal_);
  goal_publisher_.publish(goal_);
  path_publisher_.publish(path_);
}

void JoystickGoalNode::generateCVMap(nav_msgs::OccupancyGrid map)
{
  cv::Mat map_cv;
  int Cols = map.info.width;
  int Rows = map.info.height;
  map_cv.create(Rows, Cols, CV_8U);
  for (int i = 0; i < Rows; i++)
  {
    for (int j = 0; j < Cols; j++)
    {
      if (round(map.data[i * Cols + j]) >= 0)
      {
        map_cv.at<uchar>(i, j) = map.data[i * Cols + j];
      }
      else
      {
        map_cv.at<uchar>(i, j) = 100;
      }
    }
  }
  map_cv_ = map_cv;
}

geometry_msgs::Point JoystickGoalNode::goalLineOfSight(double x_goal, double y_goal, double x_agent, double y_agent)
{
  geometry_msgs::PoseStamped target;
  target.pose.orientation = goal_.pose.orientation;
  target.header = goal_.header;
  path_.header = goal_.header;
  path_.poses.clear();
  // Start path from agent position
  double x_target = x_agent;
  double y_target = y_agent;
  // Calculate the number of (fixed-resolution) intervals required  to get to the goal.
  double intervals = ceil(sqrt(pow((x_goal - x_agent), 2) + pow((y_goal - y_agent), 2)) / path_resolution_);
  // Steps along x & y axis in robot frame.
  double dx = (x_goal - x_agent) / intervals;
  double dy = (y_goal - y_agent) / intervals;
  // Walk along the path and stop when obstacle or map edge are encountered.
  for (int i = 0; i < int(intervals); i++)
  {
    // Next point along path.
    x_target += dx;
    y_target += dy;
    // Corresponding row and col index of costmap
    int index_col = ceil((x_target - origin_x_) / map_resolution_) - 1;
    int index_row = ceil((y_target - origin_y_) / map_resolution_) - 1;
    // Check whether break conditions (obstacle or map edge) are found.
    if (checkForCollision(index_row, index_col) || checkMapBoundary(index_row, index_col))
    {
      // Take a step back to feasible area.
      x_target -= dx;
      y_target -= dy;
      target.pose.position.x = x_target;
      target.pose.position.y = y_target;
      break;
    }
    target.pose.position.x = x_target;
    target.pose.position.y = y_target;
    path_.poses.push_back(target);
  }

  return target.pose.position;
}

bool JoystickGoalNode::checkForCollision(int index_row, int index_col)
{
  if ((map_cv_.at<uchar>(index_row + 0, index_col + 0) > coll_threshold_ && map_cv_.at<uchar>(index_row + 0, index_col + 0) <= 100) || (map_cv_.at<uchar>(index_row + 0, index_col - 1) > coll_threshold_ && map_cv_.at<uchar>(index_row + 0, index_col - 1) <= 100) || (map_cv_.at<uchar>(index_row + 0, index_col + 1) > coll_threshold_ && map_cv_.at<uchar>(index_row + 0, index_col + 1) <= 100) || (map_cv_.at<uchar>(index_row - 1, index_col + 0) > coll_threshold_ && map_cv_.at<uchar>(index_row - 1, index_col + 0) <= 100) || (map_cv_.at<uchar>(index_row - 1, index_col - 1) > coll_threshold_ && map_cv_.at<uchar>(index_row - 1, index_col - 1) <= 100) || (map_cv_.at<uchar>(index_row - 1, index_col + 1) > coll_threshold_ && map_cv_.at<uchar>(index_row - 1, index_col + 1) <= 100) || (map_cv_.at<uchar>(index_row + 1, index_col + 0) > coll_threshold_ && map_cv_.at<uchar>(index_row + 1, index_col + 0) <= 100) || (map_cv_.at<uchar>(index_row + 1, index_col - 1) > coll_threshold_ && map_cv_.at<uchar>(index_row + 1, index_col - 1) <= 100) || (map_cv_.at<uchar>(index_row + 1, index_col + 1) > coll_threshold_ && map_cv_.at<uchar>(index_row + 1, index_col + 1) <= 100))
    return true; // collision
  else
    return false; // no collision
}

bool JoystickGoalNode::checkMapBoundary(int index_row, int index_col)
{
  if (index_row > map_rows_ - 1 || index_row < 0 || index_col > map_cols_ - 1 || index_col < 0)
  {
    ROS_WARN("map index exceeded map boundary.");
    ROS_INFO_STREAM("index_row: " << index_row << "map_rows: " << map_rows_);
    ROS_INFO_STREAM("index_col: " << index_col << "map_cols: " << map_cols_);
    return true; // outside boundary
  }
  else
    return false; // inside boundary
}

bool JoystickGoalNode::serviceCallback(std_srvs::SetBoolRequest &request,
                                       std_srvs::SetBoolResponse &response)
{
  node_active_ = request.data;
  response.success = true;
  ROS_WARN_STREAM("Shared Control Goal Generator active: " << node_active_);
  return true;
}

} // namespace joystick_goal_node
