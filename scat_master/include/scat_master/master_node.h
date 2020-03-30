#ifndef MASTER_NODE_H
#define MASTER_NODE_H

#include <ros/ros.h>
// Standard messages
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <scat_msgs/SetMode.h>
// C++
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

namespace master_node
{

class MasterNode
{
public:
  // Constructor
  MasterNode(ros::NodeHandle &nodeHandle);

  // Destructor
  virtual ~MasterNode();

private:
  //// Node Methods ////
  bool readParameters();
  void timerCallback(const ros::TimerEvent &);
  // Publish results used to publish current mode.
  void publishResults();  
  // Callback for service, can be used to set mode. 
  void setmodeCallback(const std_msgs::String &mode);
  // Callback for service, can be used to get current mode. 
  bool getmodeCallback(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);
  // joystick input callback, can be used to switch to shared/manual as soon as user gives any input. 
  void userinputCallback(const std_msgs::Float32MultiArray &msg_userinput);
  // callback for global goal, published to move_base. System switches to autonomous mode and starts to go to goal.
  void goalCallback(const geometry_msgs::PoseStamped &goal);

  //// Algorithm Methods ////
  void changeMode(std::string mode);
  void changeToAutonomous();
  void changeToShared();
  void changeToManual();

  //// Variables ////
  std::string current_mode_ = "unknown";
  std_srvs::SetBool activate_, deactivate_; 

  //// Node Parameters ////
  // publish frequency
  float publish_frequency_ = 10;
  // threshold the normalized joystick value has to exceed to switch to manual/shared control
  float switch_threshold_ = 0.6;
  // parameter determining which global planner is used
  std::string base_global_planner_ = "global_planner/GlobalPlanner";
  // parameter determining which local planner is used
  std::string base_local_planner_ = "base_local_planner/TrajectoryPlannerROS";
  // parameter determining which type of shared control is used. 
  std::string sc_type_ = "dwa";

  // ROS objects
  ros::NodeHandle &node_handle_;
  ros::Timer timer_;
  ros::Subscriber userinput_subscriber_, goal_subscriber_, mode_subscriber_;
  ros::Publisher mode_publisher_, cancelgoal_publisher_;
  ros::ServiceServer mode_service_;
  // Service clients that are used to activate / deactivate other nodes. 
  ros::ServiceClient dwa_client_, mpc_client_, traj_client_, goal_client_, velfilter_client_;
};

} // namespace scat_master_node
#endif