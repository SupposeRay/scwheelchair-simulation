#include "sparse_voronoi/sparse_voronoi_node.h"

namespace sparse_voronoi
{

VoroNode::VoroNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
  // When debugging, set logger_level to Debug by changing Info to Debug. 
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  ROS_DEBUG("Launching Sparse Voronoi Node");

  if (!readParameters())
  {
    ROS_ERROR("Could not load parameters.");
    ros::requestShutdown();
  }
  
  base_header_.frame_id = base_frame_id_;
  base_header_.stamp = ros::Time::now();

  voronoi_algorithm_.initialize();
  voronoi_algorithm_.obstacles_.header = base_header_;

  // Subscribers
  laser_obstacle_subscriber_ = node_handle_.subscribe("/scan_obstacles", 1, &VoroNode::laserObjectCallback, this);
  cam_obstacle_subscriber_ = node_handle_.subscribe("/test_obst_msg", 1, &VoroNode::camObjectCallback, this);
  userinput_subscriber_ = node_handle_.subscribe("/user/cmd_vel", 1, &VoroNode::userinputCallback, this);
  position_subscriber_ = node_handle_.subscribe("/amcl_pose", 1, &VoroNode::positionCallback, this);
  target_subscriber_ = node_handle_.subscribe("/sparse_voronoi/targets", 1, &VoroNode::targetCallback, this);

  // Timer
  timer_ = node_handle_.createTimer(ros::Duration(1 / calc_frequency_), &VoroNode::timerCallback, this);
  
  // Publishers
  if (voronoi_algorithm_.pub_markers_)
  {
    vorolines_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("voronoi_lines", 1);
    delaunaylines_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("delaunay_lines", 1);
  }

  voropath_publisher_ = node_handle_.advertise<nav_msgs::Path>("voro_path", 1);
  interppath_publisher_ = node_handle_.advertise<nav_msgs::Path>("interp_path", 1);
  fullpath_publisher_ = node_handle_.advertise<nav_msgs::Path>("full_path", 1);
  userpath_publisher_ = node_handle_.advertise<nav_msgs::Path>("user_path", 1);

  if (interface_movebase_)
    goal_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("goal", 1);

  if  (node_active_)
  {
    while (!(laser_obstacles_received_ || cam_obstacles_received_))
    {
      ROS_INFO("sparse voronoi waiting for obstacle data");
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
    if (voronoi_algorithm_.use_dynamic_reward_)
    {
      while (!pos_received_)
      {
        ROS_INFO("sparse voronoi waiting for position data");
        ros::Duration(1).sleep();
        ros::spinOnce();
      }
    }
  }
  t_prev_ = ros::Time::now();
  ROS_DEBUG("Sparse Voronoi Node succesfully launched");
}

VoroNode::~VoroNode() {}

bool VoroNode::readParameters()
{
  // Voronoi Node parameters
  bool debug = false;
  if (!node_handle_.getParam("debug", debug))
    ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);  
  if (!node_handle_.getParam("calc_frequency", calc_frequency_))
    ROS_WARN_STREAM("Parameter calc_frequency not set for sparse_voronoi. Using default setting: " << calc_frequency_);
  if (!node_handle_.getParam("pub_frequency", pub_frequency_))
    ROS_WARN_STREAM("Parameter pub_frequency not set for sparse_voronoi. Using default setting: " << pub_frequency_);    
  if (!node_handle_.getParam("base_frame_id", base_frame_id_))
    ROS_WARN_STREAM("Parameter base_frame_id not set for sparse_voronoi. Using default setting: " << base_frame_id_);
  if (!node_handle_.getParam("fixed_frame_id", fixed_frame_id_))
    ROS_WARN_STREAM("Parameter fixed_frame_id not set for sparse_voronoi. Using default setting: " << fixed_frame_id_);
  if (!node_handle_.getParam("prediction_horizon", voronoi_algorithm_.prediction_horizon_))
    ROS_WARN_STREAM("Parameter prediction_horizon not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.prediction_horizon_);
  if (!node_handle_.getParam("sampling_time", voronoi_algorithm_.time_interval_))
    ROS_WARN_STREAM("Parameter sampling_time not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.time_interval_);
  if (!node_handle_.getParam("obst_range", obst_range_))
    ROS_WARN_STREAM("Parameter obst_range not set for sparse_voronoi. Using default setting: " << obst_range_);
  if (!node_handle_.getParam("interface_movebase", interface_movebase_))
    ROS_WARN_STREAM("Parameter interface_movebase not set for sparse_voronoi. Using default setting: " << interface_movebase_);
  if (!node_handle_.getParam("node_active", node_active_))
    ROS_WARN_STREAM("Parameter node_active not set for sparse_voronoi. Using default setting: " << node_active_); 
  // Voronoi Algorithm Object parameters
  if (!node_handle_.getParam("pub_markers", voronoi_algorithm_.pub_markers_))
    ROS_WARN_STREAM("Parameter pub_markers not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.pub_markers_);
  if (!node_handle_.getParam("tri_method", voronoi_algorithm_.tri_method_))
    ROS_WARN_STREAM("Parameter tri_method not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.tri_method_);
  if (!node_handle_.getParam("mdp_method", voronoi_algorithm_.mdp_method_))
    ROS_WARN_STREAM("Parameter mdp_method not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.mdp_method_);
  if (!node_handle_.getParam("cmd_buffer", voronoi_algorithm_.cmd_buffersize_))
    ROS_WARN_STREAM("Parameter cmd_buffer not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.cmd_buffersize_);
  if (!node_handle_.getParam("max_it", voronoi_algorithm_.max_it_))
    ROS_WARN_STREAM("Parameter max_it not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.max_it_);
  if (!node_handle_.getParam("gamma", voronoi_algorithm_.gamma_))
    ROS_WARN_STREAM("Parameter gamma not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.gamma_);
  if (!node_handle_.getParam("alpha", voronoi_algorithm_.alpha_))
    ROS_WARN_STREAM("Parameter alpha not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.alpha_);
  if (!node_handle_.getParam("min_it", voronoi_algorithm_.min_it_))
    ROS_WARN_STREAM("Parameter min_it not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.min_it_);
  if (!node_handle_.getParam("it_err_thresh", voronoi_algorithm_.it_err_thresh_))
    ROS_WARN_STREAM("Parameter it_err_thresh not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.it_err_thresh_);
  if (!node_handle_.getParam("min_gap", voronoi_algorithm_.min_gap_))
    ROS_WARN_STREAM("Parameter min_gap not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.min_gap_);
  if (!node_handle_.getParam("cost_angle", voronoi_algorithm_.cost_angle_))
    ROS_WARN_STREAM("Parameter cost_angle not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.cost_angle_); 
  if (!node_handle_.getParam("cost_angle2", voronoi_algorithm_.cost_angle2_))
    ROS_WARN_STREAM("Parameter cost_angle2 not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.cost_angle2_);       
  if (!node_handle_.getParam("use_dynamic_voronoi", voronoi_algorithm_.use_dynamic_voronoi_))
    ROS_WARN_STREAM("Parameter use_dynamic_voronoi not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_voronoi_);  
  if (!node_handle_.getParam("use_dynamic_constraint", voronoi_algorithm_.use_dynamic_constraint_))
    ROS_WARN_STREAM("Parameter use_dynamic_constraint not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_constraint_);    
  if (!node_handle_.getParam("max_path_angle", voronoi_algorithm_.max_path_angle_))
    ROS_WARN_STREAM("Parameter max_path_angle not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.max_path_angle_);
  if (!node_handle_.getParam("max_nr_states", voronoi_algorithm_.max_nr_states_))
    ROS_WARN_STREAM("Parameter max_nr_states not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.max_nr_states_);    
  if (!node_handle_.getParam("use_dynamic_reward", voronoi_algorithm_.use_dynamic_reward_))
    ROS_WARN_STREAM("Parameter use_dynamic_reward not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_dynamic_reward_);    
  if (!node_handle_.getParam("use_targets", voronoi_algorithm_.use_targets_))
    ROS_WARN_STREAM("Parameter use_targets not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.use_targets_);    
 
  if (voronoi_algorithm_.use_dynamic_voronoi_)
  {
    if (!node_handle_.getParam("voronoi_horizon", voronoi_algorithm_.voronoi_horizon_))
      ROS_WARN_STREAM("Parameter voronoi_horizon not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.voronoi_horizon_);    
    if (!node_handle_.getParam("voronoi_dt", voronoi_algorithm_.voronoi_dt_))
      ROS_WARN_STREAM("Parameter voronoi_dt not set for sparse_voronoi. Using default setting: " << voronoi_algorithm_.voronoi_dt_);       
  }
  voronoi_algorithm_.obst_range_ = obst_range_;
  voronoi_algorithm_.fixed_frame_id_ = fixed_frame_id_;
  voronoi_algorithm_.base_frame_id_ = base_frame_id_;

  // When debugging, set logger_level to Debug by changing Info to Debug. 
  if (debug)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  return true;
}

void VoroNode::userinputCallback(const geometry_msgs::Twist::ConstPtr &msg_userinput)
{
  cmd_vel_.linear.x = msg_userinput->linear.x;
  cmd_vel_.angular.z = msg_userinput->angular.z;
  cmd_received_ = true;
  

  t_now_ = ros::Time::now();
  ros::Duration dt = t_now_ - t_prev_;
  if (dt.toSec() > 1/pub_frequency_)
  {

    if (fabs(cmd_vel_.linear.x) > v_threshold_ || fabs(cmd_vel_.angular.z) > w_threshold_)
    {
      // for accurate calculation time checking:
      std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now();
      
      // Calculate the MDP to get the best trajectory based on user input
      voronoi_algorithm_.processMDP(global_pose2D_, cmd_vel_);
      publishResults();

      std::chrono::system_clock::time_point t_end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_time = t_end - t_start;
      ROS_DEBUG_STREAM("MCP succesfully calculated. Calculation time: " << elapsed_time.count());
      t_prev_ = t_now_;
    }

  }
}

void VoroNode::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_position)
{
  global_pose2D_.x = msg_position->pose.pose.position.x;
  global_pose2D_.y = msg_position->pose.pose.position.y;
  global_pose2D_.theta = tf::getYaw(msg_position->pose.pose.orientation);
  pos_received_ = true;
}

void VoroNode::publishResults()
{
  nav_msgs::Path interp_path = rosmsg::convertPath(voronoi_algorithm_.interp_path_,base_header_);
  voropath_publisher_.publish(voronoi_algorithm_.voro_path_);
  interppath_publisher_.publish(interp_path);

  if (interface_movebase_ && cmd_received_ && std::fabs(cmd_vel_.linear.x) > 0)
    goal_publisher_.publish(interp_path.poses.back());    
  
  fullpath_publisher_.publish(voronoi_algorithm_.full_path_);

  userpath_publisher_.publish(diff_drive::makePath(cmd_vel_,voronoi_algorithm_.prediction_horizon_,voronoi_algorithm_.time_interval_,rosmsg::makeHeader(base_frame_id_,ros::Time::now())));  
}

void VoroNode::timerCallback(const ros::TimerEvent &)
{
  if (laser_obstacles_received_ || cam_obstacles_received_)
  {
    // ROS_DEBUG_STREAM("Starting Voronoi Process");
    std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now();
    base_header_.stamp = ros::Time::now();
    
    // Combine obstacle messages (e.g. from laser and camera)
    processObstacles();    

    // // Perform triangulation, Construct Voronoi diagram and execute value iteration to find optimal path.
    voronoi_algorithm_.processVoronoi(base_header_);

    // publish markers
    if (voronoi_algorithm_.pub_markers_)
    {
      vorolines_publisher_.publish(voronoi_algorithm_.voronoi_lines_);
      delaunaylines_publisher_.publish(voronoi_algorithm_.delaunay_lines_);
    }

    std::chrono::system_clock::time_point t_end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = t_end - t_start;
    ROS_DEBUG_STREAM("Voronoi diagram constructed. Voronoi calculation time: " << elapsed_time.count());
  }
}

void VoroNode::laserObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects)
{
  if (node_active_)
  {
    // automatically transform into base_frame
    laser_object_msg_ = obst_utils::transformObjectList(*msg_objects, base_frame_id_, tf_listener_);
    laser_obstacles_received_ = true;
    // ROS_INFO_STREAM("laser obstacles received: " << laser_object_msg_.objects.size());
  }
}

void VoroNode::camObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects)
{
  if (node_active_)
  {
    // automatically transform into base_frame
    cam_object_msg_ = obst_utils::transformObjectList(*msg_objects, base_frame_id_, tf_listener_);
    cam_obstacles_received_ = true;
    // ROS_INFO_STREAM("cam obstacles received: " << cam_object_msg_.objects.size());
  }
}

void VoroNode::targetCallback(const geometry_msgs::PoseArray::ConstPtr &msg_targets)
{
  if (node_active_ && voronoi_algorithm_.use_targets_)
  {
    // automatically transform into base_frame
    voro_targets_ = tf_utils::transformPoseArray(*msg_targets, base_frame_id_, tf_listener_);
    targets_received_ = true;
    // ROS_INFO_STREAM("targets received: " << voro_targets_poses.size());
  }
}

void VoroNode::processObstacles()
{
  // ROS_DEBUG_STREAM("Processing (merging) Obstacles");
  // obstacles_pts_ is a vector containing obstacles, each described by a vector of (ros) points. 
  voronoi_algorithm_.obstacles_.objects.clear();
  voronoi_algorithm_.obstacles_.header.stamp = ros::Time::now();
  if (laser_obstacles_received_)
  {
    for (auto &object : laser_object_msg_.objects)
    {
      voronoi_algorithm_.obstacles_.objects.push_back(object);
    }
  }
  if (cam_obstacles_received_)
  {
    for (auto &object : cam_object_msg_.objects)
    {
      voronoi_algorithm_.obstacles_.objects.push_back(object);
    }
  }
  if (voronoi_algorithm_.use_targets_)
  {
    voronoi_algorithm_.voro_targets_ = voro_targets_;
  }
}

} // namespace sparse_voronoi
