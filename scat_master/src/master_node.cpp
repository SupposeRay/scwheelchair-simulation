#include <scat_master/master_node.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "master_node");
  ros::NodeHandle node_handle("~");
  master_node::MasterNode MasterNode(node_handle);
  ros::spin();

  return 0;
}

namespace master_node
{

MasterNode::MasterNode(ros::NodeHandle &nodeHandle)
    : node_handle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("MasterNode: Could not load parameters.");
    ros::requestShutdown();
  }

  activate_.request.data = true;
  deactivate_.request.data = false;

  // Publishers
  mode_publisher_ = node_handle_.advertise<std_msgs::String>("current_mode", 1);

  // Subscribers
  mode_service_ = node_handle_.advertiseService("get_mode", &MasterNode::getmodeCallback, this);
  mode_subscriber_ = node_handle_.subscribe("set_mode", 1, &MasterNode::setmodeCallback, this);
  
  userinput_subscriber_ = node_handle_.subscribe("/user/joy", 1, &MasterNode::userinputCallback, this);
  goal_subscriber_ = node_handle_.subscribe("/move_base_simple/goal", 1, &MasterNode::goalCallback, this);
  timer_ = node_handle_.createTimer(ros::Duration(1/publish_frequency_), &MasterNode::timerCallback, this);
  cancelgoal_publisher_ = node_handle_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

  // activation service clients
  dwa_client_ = node_handle_.serviceClient<std_srvs::SetBool>("/shared_dwa/activation_service");  
  mpc_client_ = node_handle_.serviceClient<std_srvs::SetBool>("/shared_mpc/activation_service");  
  traj_client_ = node_handle_.serviceClient<std_srvs::SetBool>("/trajectory_generator/activation_service");  
  goal_client_ = node_handle_.serviceClient<std_srvs::SetBool>("/joystick_goal/activation_service");  
  velfilter_client_ = node_handle_.serviceClient<scat_msgs::SetMode>("/vel_safety_filter/mode_service");  

  ROS_DEBUG("Master Node Constructor finished succesfully");
}

MasterNode::~MasterNode() {}

bool MasterNode::readParameters()
{
  if (!node_handle_.getParam("base_global_planner", base_global_planner_))
    ROS_WARN_STREAM("Parameter base_global_planner not set for scat_master. Using default setting: " << base_global_planner_);
  if (!node_handle_.getParam("base_local_planner", base_local_planner_))
    ROS_WARN_STREAM("Parameter base_local_planner not set for scat_master. Using default setting: " << base_local_planner_);
  if (!node_handle_.getParam("switch_threshold", switch_threshold_))
    ROS_WARN_STREAM("Parameter switch_threshold not set for scat_master. Using default setting: " << switch_threshold_);
  if (!node_handle_.getParam("sc_type", sc_type_))
    ROS_WARN_STREAM("Parameter sc_type not set for scat_master. Using default setting: " << sc_type_);


  // When debugging, set logger_level to Debug by changing Info to Debug.
  bool debug = false;
  if (!node_handle_.getParam("debug", debug))
      ROS_WARN_STREAM("Parameter debug not set for sparse_voronoi. Using default setting: " << debug);        
  if (debug)
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  else
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  // Initialize mode
  std::string set_mode = "autonomous";
  node_handle_.getParam("mode", set_mode);
  changeMode(set_mode);

  return true;
}

void MasterNode::timerCallback(const ros::TimerEvent &)
{
  publishResults();
}
  
// Publish results used to publish current mode.
void MasterNode::publishResults()
{
  std_msgs::String mode_msg;
  mode_msg.data = current_mode_;
  mode_publisher_.publish(mode_msg);
}

// Callback for service, can be used to get current mode. 
bool MasterNode::getmodeCallback(std_srvs::TriggerRequest &request,
                                 std_srvs::TriggerResponse &response)
{
  response.message = current_mode_;
  if (current_mode_ == "unknown")
    response.success = false;
  else
    response.success = true;
  return true;
}
  
// Callback for service, can be used to set mode. 
void MasterNode::setmodeCallback(const std_msgs::String &mode)
{
  changeMode(mode.data);
}

// callback for global goal, published to move_base. System switches to autonomous mode and starts to go to goal.
void MasterNode::goalCallback(const geometry_msgs::PoseStamped &goal)
{
  if (current_mode_ != "autonomous" && !(sc_type_ == "trajectory" || sc_type_ ==  "goal"))
    changeMode("autonomous");
}

// When in Autonomous control mode, this method allows to switch to Shared or Manual control mode when the joystick is used. 
void MasterNode::userinputCallback(const std_msgs::Float32MultiArray &msg_userinput)
{
  if (fabs(msg_userinput.data[0]) > switch_threshold_  && current_mode_ == "autonomous")
      changeMode("shared");
}

void MasterNode::changeMode(std::string mode)
{
  ROS_DEBUG("Current mode: %s", current_mode_.c_str());
  ROS_DEBUG("Changing mode to: %s", mode.c_str());
  // No C++ library for dynamic reconfigure has been implemented yet.
  if (mode == "autonomous")
  {
    changeToAutonomous();
  }
  else if (mode == "shared")
  {
    changeToShared();
  }
  else if (mode == "manual")
  {
    changeToManual();
  }
  else if (mode == "docked")
  {
    current_mode_ = mode;
    ROS_INFO("Changed mode to Docked");
  }
  else
  {
    ROS_WARN("Wheelchair Mode setting change attempt via master_node failed: unknown mode");
    current_mode_ = "unknown";
  }
  ROS_DEBUG("Succesfully changed mode");
}

void MasterNode::changeToAutonomous()
{
  scat_msgs::SetMode mode_srv;
  mode_srv.request.mode = "autonomous";
  velfilter_client_.call(mode_srv);

  if (sc_type_ == "trajectory")
  {
    traj_client_.call(deactivate_);
    system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner global_planner/GlobalPlanner");
  }
  else if (sc_type_ == "dwa") 
  {
    dwa_client_.call(deactivate_);
  }
  else if (sc_type_ == "mpc")
  {
    mpc_client_.call(deactivate_);
  }
  else if (sc_type_ == "goal")
  {
    goal_client_.call(deactivate_);
  }
  else
  {
    ROS_ERROR_STREAM("sc type not recognized.");
  }
  
  current_mode_ = "autonomous";
  ROS_INFO("Changed mode to Autonomous");
  ROS_WARN_STREAM("Activated Autonomous Navigation");
}

void MasterNode::changeToShared()
{
  actionlib_msgs::GoalID cancelgoal;
  cancelgoal_publisher_.publish(cancelgoal);
  ROS_WARN_STREAM("DeActivated Autonomous Navigation");

  scat_msgs::SetMode mode_srv;
  mode_srv.request.mode = "shared";
  velfilter_client_.call(mode_srv);

  if (sc_type_ == "trajectory")
  {
    system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner global_planner_plugin/GlobalPlannerPlugin");
    traj_client_.call(activate_);
    ROS_WARN_STREAM("Activated Shared Control Trajectory Generator");
  }
  else if (sc_type_ == "dwa") 
  {
    dwa_client_.call(activate_);
    ROS_WARN_STREAM("Activated Shared DWA");
  }
  else if (sc_type_ == "mpc")
  {
    mpc_client_.call(activate_);
    ROS_WARN_STREAM("Activated Shared MPC");
  }
  else if (sc_type_ == "goal")
  {
    goal_client_.call(activate_);
    ROS_WARN_STREAM("Activated Shared Control Goal Generator");
  }
  else
  {
    ROS_ERROR_STREAM("sc type not recognized.");
  }

  current_mode_ = "shared";
  ROS_INFO("Changed mode to Shared Control");
}

void MasterNode::changeToManual()
{
  actionlib_msgs::GoalID cancelgoal;
  cancelgoal_publisher_.publish(cancelgoal);    
  ROS_WARN_STREAM("DeActivated Autonomous Navigation");

  scat_msgs::SetMode mode_srv;
  mode_srv.request.mode = "manual";
  velfilter_client_.call(mode_srv);

  if (sc_type_ == "trajectory")
  {
    traj_client_.call(deactivate_);
    system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner global_planner/GlobalPlanner");
  }
  else if (sc_type_ == "dwa") 
  {
    dwa_client_.call(deactivate_);
  }
  else if (sc_type_ == "mpc")
  {
    mpc_client_.call(deactivate_);
  }
  else if (sc_type_ == "goal")
  {
    goal_client_.call(deactivate_);
  }
  else
  {
    ROS_ERROR_STREAM("sc type not recognized.");
  }

  current_mode_ = "manual";
  ROS_INFO("Changed mode to Manual");  
}

} // namespace scatwheel_master_node
