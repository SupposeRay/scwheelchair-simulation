#include <global_planner_plugin/global_planner_plugin.h>

//Declare this planner as a BaseGlobalPlanner plugin class
PLUGINLIB_EXPORT_CLASS( global_planner_plugin::GlobalPlannerPlugin, 
                        nav_core::BaseGlobalPlanner)


namespace global_planner_plugin
{
using namespace std;
using namespace ros;
// Constructor
GlobalPlannerPlugin::GlobalPlannerPlugin()
    : initialized_(false), costmap_ros_(NULL)
{
  ROS_WARN("Creating GlobalPlannerPlugin Object");
}

// Standard Constructor
GlobalPlannerPlugin::GlobalPlannerPlugin(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : initialized_(false), costmap_ros_(NULL)
{
  initialize(name, costmap_ros);
  ROS_WARN("Initialize GlobalPlannerPlugin");
}
// Standard Destructor
GlobalPlannerPlugin::~GlobalPlannerPlugin() {}

void GlobalPlannerPlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/" + name);
    sub_trajectory_ = private_nh.subscribe("/global_planner_plugin/trajectory", 1, &GlobalPlannerPlugin::trajectoryCallback, this);
    pub_plan_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    pub_goal_ = private_nh.advertise<geometry_msgs::PoseStamped>("goal", 1);

    ROS_WARN("Initialized planner named %s", name.c_str());
    initialized_ = true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized");
  }
  ROS_INFO("Custom Planner Plugin is initalized.");
}

bool GlobalPlannerPlugin::makePlan(const geometry_msgs::PoseStamped &start,
                                    const geometry_msgs::PoseStamped &goal,
                                    std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO("makePlan");

  if (!initialized_)
  {
    ROS_ERROR("Global planner is not initialized");
    return false;
  }


  plan.clear();
  if (trajectory_received_)
  {
    ROS_INFO("Following Trajectory");
    for (int i = 0; i < path_.poses.size(); i++)
    {
      plan.push_back(path_.poses[i]);
    }
    std::cout << "plan size: " << plan.size() << std::endl;
  }
  else
  {
    ROS_WARN("Received goal but no trajectory.");
    // plan.push_back(goal);
  }

  pub_plan_.publish(path_);
  pub_goal_.publish(goal);
  return true;
}

void GlobalPlannerPlugin::trajectoryCallback(const nav_msgs::PathConstPtr &trajectory)
{
  trajectory_received_ = true;
  path_.poses.clear();
  path_.header = trajectory->header;
  path_.poses = trajectory->poses;
}
}; // namespace probabilistic_planner
