#include "vel_safety_filter/vel_safety_filter_node.h"

namespace vel_safety_filter
{

//! Constructor
VelFilterNode::VelFilterNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
  ROS_DEBUG("Launching Velocity command filter node");

  if (!readParameters() || !readBotParameters())
  {
    ROS_ERROR("Could not load parameters.");
    ros::requestShutdown();
  }

  //! Subscribers & Publishers
  if (obst_type_ == "scan")
    obst_subscriber_ = node_handle_.subscribe("/scan", 1, &VelFilterNode::scanCallback, this);
  else if (obst_type_ == "obstmsg")
    obst_subscriber_ = node_handle_.subscribe("/obstacles", 1, &VelFilterNode::obstCallback, this);
  else if (obst_type_ == "cloud")
    obst_subscriber_ = node_handle_.subscribe("/pointcloud", 1, &VelFilterNode::cloudCallback, this);

  if (vel_type_ == "odom")
      odom_subscriber_ = node_handle_.subscribe("/odom", 1, &VelFilterNode::odomCallback, this);
  else if (vel_type_ == "twist")
      vel_subscriber_ = node_handle_.subscribe("/velocity", 1, &VelFilterNode::velCallback, this);
    
  mode_service_ = node_handle_.advertiseService("mode_service", &VelFilterNode::serviceCallback, this);

  if (use_col_filter_)
  {
    while (!(scan_received_ || cloud_received_ || obst_received_))
    {
      ROS_INFO("vel_safety_filter waiting for obstacle data");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
    while (!vel_received_)
    {
      ROS_INFO("vel_safety_filter waiting for velocity");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
    ROS_INFO("vel_safety_filter all data received");
  }
  if (use_vel_filter_ || use_acc_filter_)
  {
    vel_utils_.setParams(
        v_min_, v_max_,
        v_max_neg_,
        w_min_, w_max_,
        v_acc_, v_dec_,
        w_acc_, w_dec_,
        v_threshold_, w_threshold_,
        sampling_time_, base_width_);
  }
  usercmd_subscriber_ = node_handle_.subscribe("/user/cmd_vel", 1, &VelFilterNode::userCmdCallback, this);
  ctrlcmd_subscriber_ = node_handle_.subscribe("/control/cmd_vel", 1, &VelFilterNode::controlCmdCallback, this);
  cmd_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  shape_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("bot_shape", 1);
  path_publisher_ = node_handle_.advertise<nav_msgs::Path>("checked_path", 1);
  timer_ = node_handle_.createTimer(ros::Duration(time_out_), &VelFilterNode::timerCallback, this);

  t_prev_ = ros::Time::now();
  vel_agent_.linear.x = 0;
  vel_agent_.angular.z = 0;

  if (use_col_filter_ && collision_method_ == "footprint")
  {
    footprint_marker_.type = visualization_msgs::Marker::LINE_LIST;
    footprint_marker_.color.g = 1.0;
    footprint_marker_.color.a = 1.0;
    footprint_marker_.scale.x = 0.02;
  }
}
//! Destructor
VelFilterNode::~VelFilterNode() {}

bool VelFilterNode::readBotParameters()
{
  if (!node_handle_.getParam("v_min", v_min_))
    ROS_WARN_STREAM("Parameter v_min not set for shared_mpc. Using default setting: " << v_min_);
  if (!node_handle_.getParam("v_max", v_max_))
    ROS_WARN_STREAM("Parameter v_max not set for shared_mpc. Using default setting: " << v_max_);
  if (!node_handle_.getParam("v_max_neg", v_max_neg_))
    ROS_WARN_STREAM("Parameter v_max_neg not set for shared_mpc. Using default setting: " << v_max_neg_);
  if (!node_handle_.getParam("v_threshold", v_threshold_))
    ROS_WARN_STREAM("Parameter v_threshold not set for shared_mpc. Using default setting: " << v_threshold_);
  if (!node_handle_.getParam("w_max", w_max_))
    ROS_WARN_STREAM("Parameter w_max not set for shared_mpc. Using default setting: " << w_max_);
  if (!node_handle_.getParam("w_threshold", w_threshold_))
    ROS_WARN_STREAM("Parameter w_threshold not set for shared_mpc. Using default setting: " << w_threshold_);
  if (!node_handle_.getParam("acc_lim_x", v_acc_))
    ROS_WARN_STREAM("Parameter acc_lim_x not set for shared_mpc. Using default setting: " << v_acc_);
  if (!node_handle_.getParam("dec_lim_x", v_dec_))
    ROS_WARN_STREAM("Parameter dec_lim_x not set for shared_mpc. Using default setting: " << v_dec_);
  if (!node_handle_.getParam("acc_lim_theta", w_acc_))
    ROS_WARN_STREAM("Parameter acc_lim_theta not set for shared_mpc. Using default setting: " << w_acc_);
  if (!node_handle_.getParam("dec_lim_theta", w_dec_))
    ROS_WARN_STREAM("Parameter dec_lim_theta not set for shared_mpc. Using default setting: " << w_dec_);
  if (!node_handle_.getParam("base_width", base_width_))
    ROS_WARN_STREAM("Parameter base_width not set for shared_mpc. Using default setting: " << base_width_);
  if (!node_handle_.getParam("robot_radius", robot_radius_))
    ROS_WARN_STREAM("Parameter robot_radius not set for shared_mpc. Using default setting: " << robot_radius_);

  if (v_max_neg_ > 0)
  {
    ROS_ERROR_STREAM("v_max_neg is larger than zero. This should be a negative number.");
    return false;
  }

  if (!node_handle_.getParam("footprint", footprint_coords_))
  {
    ROS_WARN_STREAM("Parameter footprint not set for shared_dwa. Using radius collision_method.");
    collision_method_ = "radius";
  }

  if (footprint_coords_.size() < 6)
  {
    ROS_ERROR("number of footprint segments less than 3. Unable to continue.");
    return false;
  }

  w_min_ = 2 * v_min_ / base_width_;
  robot_radius_ = robot_radius_ * footprint_inflation_;
  footprint_ = nav_utils::createPolygon2D(footprint_coords_, footprint_inflation_, 2);

  // Calculate the largest distance from center of rotation to corner point, for rotation collision hazard calculation.
  for (auto corner : footprint_)
  {
    float L = base_utils::euclideanDistance(corner);
    if (L > L_max_)
      L_max_ = L;
  }

  return true;
}

/*!
 * \brief function which reads parameters from parameterserver
 */
bool VelFilterNode::readParameters()
{

  bool debug = false;
  if (!node_handle_.getParam("debug", debug))
    ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);
  // When debugging, set logger_level to Debug by changing Info to Debug.
  if (debug)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  if (!node_handle_.getParam("use_vel_filter", use_vel_filter_))
    ROS_WARN_STREAM("Parameter use_vel_filter not set for vel_filter. Using default setting: " << use_vel_filter_);
  if (!node_handle_.getParam("use_acc_filter", use_acc_filter_))
    ROS_WARN_STREAM("Parameter use_acc_filter not set for vel_filter. Using default setting: " << use_acc_filter_);
  if (!node_handle_.getParam("use_col_filter", use_col_filter_))
    ROS_WARN_STREAM("Parameter use_col_filter not set for vel_filter. Using default setting: " << use_col_filter_);
  if (!node_handle_.getParam("fixed_frame_id", fixed_frame_id_))
    ROS_WARN_STREAM("Parameter fixed_frame_id not set for vel_filter. Using default setting: " << fixed_frame_id_);
  if (!node_handle_.getParam("base_frame_id", base_frame_id_))
    ROS_WARN_STREAM("Parameter base_frame_id not set for vel_filter. Using default setting: " << base_frame_id_);
  if (!node_handle_.getParam("collision_method", collision_method_))
    ROS_WARN_STREAM("Parameter collision_method not set for vel_filter. Using default setting: " << collision_method_);
  if (!node_handle_.getParam("mode", mode_))
    ROS_WARN_STREAM("Parameter mode not set for vel_filter. Using default setting: " << mode_);
  if (!node_handle_.getParam("footprint_inflation", footprint_inflation_))
    ROS_WARN_STREAM("Parameter footprint_inflation not set for vel_filter. Using default setting: " << footprint_inflation_);
  if (!node_handle_.getParam("vel_inflation", vel_inflation_))
    ROS_WARN_STREAM("Parameter vel_inflation not set for shared_dwa. Using default setting: " << vel_inflation_);
  if (!node_handle_.getParam("obstacle_inflation", obst_inflation_))
    ROS_WARN_STREAM("Parameter obstacle_inflation not set for vel_filter. Using default setting: " << obst_inflation_);
  if (!node_handle_.getParam("angle_res", angle_res_))
    ROS_WARN_STREAM("Parameter angle_res not set for vel_filter. Using default setting: " << angle_res_);
  if (!node_handle_.getParam("sampling_time", sampling_time_))
    ROS_WARN_STREAM("Parameter sampling_time not set for vel_filter. Using default setting: " << sampling_time_);
  if (!node_handle_.getParam("time_out", time_out_))
    ROS_WARN_STREAM("Parameter time_out not set for vel_filter. Using default setting: " << time_out_);
  if (!node_handle_.getParam("obst_type", obst_type_))
    ROS_WARN_STREAM("Parameter obst_type not set for vel_filter. Using default setting: " << obst_type_);
  if (!node_handle_.getParam("obst_range", obst_range_))
    ROS_WARN_STREAM("Parameter obst_range not set for vel_filter. Using default setting: " << obst_range_);
    if (!node_handle_.getParam("vel_type", vel_type_))
        ROS_WARN_STREAM("Parameter vel_type not set for shared_dwa. Using default setting: " << vel_type_);

  if (collision_method_ == "radius")
    obst_type_ == "scan";
    
  return true;
}

/*!
 * \details callback for cmd_vel in manual mode
 */
void VelFilterNode::userCmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd)
{
  if (mode_ == "manual")
    processCmd(*msg_cmd);
}

/*!
 * \details callback for cmd_vel in shared/autonomous mode
 */
void VelFilterNode::controlCmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd)
{
  if (mode_ == "shared" || mode_ == "autonomous")
    processCmd(*msg_cmd);
}

// Calls all the checks to be performed on the velocity command, i.e. checkAcceleration, checkVelocity and checkCollision.
void VelFilterNode::processCmd(const geometry_msgs::Twist &msg_cmd)
{
  geometry_msgs::Twist cmd_vel = msg_cmd;
  // timing parameters required for timeout safety functionality
  t_now_ = ros::Time::now();
  ros::Duration dt_ros = t_now_ - t_prev_;
  // double dt = dt_ros.toSec();
  if (use_acc_filter_)
    cmd_vel = vel_utils_.limitAcceleration(cmd_vel, vel_agent_);
  if (use_vel_filter_)
    cmd_vel = vel_utils_.limitVelocity(cmd_vel);
  if (use_col_filter_)
    cmd_vel = checkCollision(cmd_vel);
  publishResults(cmd_vel);
  t_prev_ = t_now_;
}

/*!
 * \details callback for scan
 */
void VelFilterNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan)
{
  if (obst_type_ == "scan")
  {
    if (collision_method_ == "footprint")
    {

      sensor_msgs::PointCloud cloud;
      try
      {
        tf_listener_.waitForTransform(msg_scan->header.frame_id, base_frame_id_,
                                      msg_scan->header.stamp + ros::Duration().fromSec(msg_scan->ranges.size() * msg_scan->time_increment),
                                      ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud(base_frame_id_, *msg_scan, cloud, tf_listener_);
        cloud_received_ = true;
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("Vel_filter: %s", ex.what());
        return;
      }

      point_cloud_.points.clear();
      point_cloud_.header = cloud.header;
      for (auto &point : cloud.points)
      {
        if (base_utils::euclideanDistance(point) < obst_range_)
        {
          point_cloud_.points.push_back(point);
        }
      }
    }
    else if (collision_method_ == "radius")
    {
      scan_ = *msg_scan;
      scan_received_ = true;
    }
  }
}

void VelFilterNode::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_cloud)
{
  if (obst_type_ == "cloud")
  {
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud cloud;

    try
    {
      tf_listener_.waitForTransform(msg_cloud->header.frame_id, base_frame_id_,
                                    msg_cloud->header.stamp + ros::Duration().fromSec(0.1),
                                    ros::Duration(1.0));
      pcl_ros::transformPointCloud(base_frame_id_, *msg_cloud, cloud2, tf_listener_);
      cloud_received_ = true;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Vel_filter: %s", ex.what());
      return;
    }
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2,cloud);
    point_cloud_.points.clear();
    point_cloud_.header = cloud.header;
    for (auto &point : cloud.points)
    {
      if (base_utils::euclideanDistance(point) < obst_range_ && point.z > height_threshold_)
      {
        point_cloud_.points.push_back(point);
      }
    }
  }
}

void VelFilterNode::obstCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_obst)
{
  if (obst_type_ == "obstmsg")
  {
    // obstacles_pts_ is a vector containing obstacles, each described by a vector of (ros) points.
    obstacles_pts_ = obst_utils::convertObjectListToPoints(*msg_obst, base_frame_id_, tf_listener_);
    if (obstacles_pts_.size()>0)
      obst_received_ = true;
  }
}

/*!
 * \details receives current bot velocity
 */
void VelFilterNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom)
{
  v_agent_ = msg_odom->twist.twist.linear.x;
  w_agent_ = msg_odom->twist.twist.angular.z;
  vel_agent_ = msg_odom->twist.twist;
  vel_received_ = true;
}

void VelFilterNode::velCallback(const geometry_msgs::Twist::ConstPtr &msg_vel)
{

  v_agent_ = msg_vel->linear.x;
  w_agent_ = msg_vel->angular.z;
  vel_received_ = true;
}

/*!
 * \details all publishers contained with this function
 */
void VelFilterNode::publishResults(geometry_msgs::Twist cmd_vel)
{
  cmd_publisher_.publish(cmd_vel);
  // ROS_DEBUG_STREAM("V cmd: " << cmd_vel.linear.x << " W cmd: " << cmd_vel.angular.z);
  if (use_col_filter_ && collision_method_ == "footprint")
  {
    footprint_marker_.header = rosmsg::makeHeader(base_frame_id_, t_now_);
    footprint_marker_.points = rosmsg::makeMarkerPoints(nav_utils::moveFootprint(footprint_,
                                                                                 path_.poses.back().pose.position.x,
                                                                                 path_.poses.back().pose.position.y,
                                                                                 tf::getYaw(path_.poses.back().pose.orientation)));
    shape_publisher_.publish(footprint_marker_);
    path_publisher_.publish(path_);
  }
}

geometry_msgs::Twist VelFilterNode::checkCollision(geometry_msgs::Twist cmd_vel)
{
  // float v_ref = std::max(fabs(v_agent_),fabs(cmd_vel.linear.x));
  // float w_ref = std::max(fabs(w_agent_),fabs(cmd_vel.angular.z));
  float v_ref = cmd_vel.linear.x;
  float w_ref = cmd_vel.angular.z;
  float cmd_angle = base_utils::getAngle(cmd_vel);
  // Add additional inflation to footprint, depending on velocity.
  float inflation = footprint_inflation_ + vel_inflation_ * std::max(fabs(v_ref) - v_min_, 0.0) / (v_max_ - v_min_);
  footprint_ = nav_utils::createPolygon2D(footprint_coords_, inflation, 2);
  // Check current position for collision. It could be the case that we're already close to a wall or table, and we want to get away.
  // In this case we should only allow velocities which are directed away from the collision points.
  if (collision_method_ == "radius" && scan_received_)
  {
    std::vector<double> coll_angles;
    if (nav_utils::checkRadiusForCollision(robot_radius_, scan_, 0.0, 0.0, 0.0, angle_res_, obst_inflation_, coll_angles))
    {
      bool cmd_allowed = true;
      // The following code checks whether the command cmd_vel is directed towards the collision or away.
      // cmd_vel is allowed if it is directed away from collision, and in this case no further collision check is performed.
      for (double &angle : coll_angles)
      {
        if (fabs(base_utils::shiftAngle(cmd_angle - angle)) < M_PI / 2)
        {
          cmd_allowed = false;
        }
      }
      if (cmd_allowed)
        return cmd_vel;
      // Current position is in collision, but cmd_vel is directed away from collision.
    }
  }
  else if (collision_method_ == "footprint" && (cloud_received_ || obst_received_))
  {
    // Do the same steps but with footprint method instead of radius method.
    std::vector<geometry_msgs::Point32> coll_points;
    if ((cloud_received_ && nav_utils::checkFootprintForCollision(footprint_, point_cloud_, 0, 0, 0, coll_points)) 
    || (obst_received_ && nav_utils::checkFootprintForCollision(footprint_, obstacles_pts_, 0, 0, 0, coll_points)))
    {
      bool cmd_allowed = true;
      // The following code checks whether the command cmd_vel is directed towards the collision or away.
      // cmd_vel is allowed if it is directed away from collision, and in this case no further collision check is performed.
      for (auto &point : coll_points)
      {
        if (fabs(base_utils::shiftAngle(cmd_angle - base_utils::getAngle(point))) < M_PI / 2)
        {
          cmd_allowed = false;
        }
      }
      if (cmd_allowed)
        return cmd_vel;
      // Current position is in collision, but cmd_vel is directed away from collision.
    }
  }    
  else
  {
    ROS_ERROR_STREAM("Unable to do collision check! Data not received.");
  }

  // The following code repeatedly extrapolates the predicted trajectory based on cmd_vel.
  // If it detects collision, based on either "radius" or "footprint" method and pointcloud from scan or camera, or obstacle message.

  bool collision = true;
  while (collision)
  {
    collision = false;
    float dec_time = std::max(fabs(v_ref) / v_dec_, fabs(w_ref) / w_dec_) + sqrt(2 * obst_inflation_ / std::min(v_dec_, w_dec_ * L_max_)) + 2 * sampling_time_;
    int path_samples = static_cast<int>(ceil(dec_time / sampling_time_));
    path_ = diff_drive::makePath(v_ref, w_ref, path_samples, sampling_time_, rosmsg::makeHeader(base_frame_id_, t_now_));

    if (collision_method_ == "radius" && scan_received_)
    {
      for (auto &pose : path_.poses)
      {
        if (nav_utils::checkRadiusForCollision(robot_radius_, scan_,
                                               pose.pose.position.x, pose.pose.position.y,
                                               tf::getYaw(pose.pose.orientation),
                                               angle_res_, obst_inflation_))
        {
          collision = true;
          break;
        }
      }
    }
    else if (collision_method_ == "footprint" && (cloud_received_ || obst_received_))
    {
      for (auto &pose : path_.poses)
      {
        if ((cloud_received_ && nav_utils::checkFootprintForCollision(footprint_, point_cloud_, pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation))) 
        || (obst_received_ && nav_utils::checkFootprintForCollision(footprint_, obstacles_pts_, pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation))))
        {
          collision = true;
          break;
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unable to do collision check!");
    }
    

    if (collision)
    {
      // decrease linear and angular velocities
      v_ref -= base_utils::sign(v_ref) * v_dec_ * sampling_time_;
      w_ref -= base_utils::sign(w_ref) * w_dec_ * sampling_time_;

      if (fabs(v_ref) < v_min_ || base_utils::sign(v_ref) != base_utils::sign(cmd_vel.linear.x))
        v_ref = 0;
      if (fabs(w_ref) < w_min_ || base_utils::sign(w_ref) != base_utils::sign(cmd_vel.angular.z))
        w_ref = 0;
      if (fabs(v_ref) < v_min_ && fabs(w_ref) < w_min_)
        collision = false;
      ROS_WARN_STREAM("Collision detected. Decreasing velocity to v = " << v_ref << " w = " << w_ref);
    }
  }

  // ROS_DEBUG_STREAM("v_agent: " << v_agent_ << " v_cmd: " << cmd_vel.linear.x);
  // ROS_DEBUG_STREAM("w_agent: " << w_agent_ << " w_cmd: " << cmd_vel.angular.z);
  // ROS_DEBUG_STREAM("v_dist: " << obst_inflation_/v_ref << " w_dist: " << obst_inflation_/w_ref*L_max_);
  // ROS_DEBUG_STREAM("v_ref: " << v_ref);
  // ROS_DEBUG_STREAM("w_ref: " << w_ref);
  // ROS_DEBUG_STREAM("Deceleration time: " << dec_time);
  // ROS_DEBUG_STREAM("Path samples: " << path_samples);
  // ROS_DEBUG_STREAM("Path straight-line distance: " << base_utils::euclideanDistance(path_.poses.back().pose.position));

  // if (fabs(v_ref) < fabs(cmd_vel.linear.x) || fabs(w_ref) < fabs(cmd_vel.angular.z))
  //   // some velocity limiting has occured, brake
  // return rosmsg::makeTwist(v_ref,w_ref);
  // else
  //   return cmd_vel;

  return rosmsg::makeTwist(v_ref, w_ref);
}

bool VelFilterNode::serviceCallback(scat_msgs::SetModeRequest &request,
                                    scat_msgs::SetModeResponse &response)
{
  mode_ = request.mode;
  response.succes = true;
  ROS_WARN_STREAM("Vel filter mode set to: " << mode_);
  return true;
}

/*!
 * \brief callback for timer
 */
void VelFilterNode::timerCallback(const ros::TimerEvent &)
{
  ros::Time t_timer = ros::Time::now();
  ros::Duration t_lag = t_timer - t_prev_;
  if (t_lag.toSec() > time_out_)
  {
    processCmd(rosmsg::makeTwist(0, 0));
  }
}

} // namespace vel_safety_filter
