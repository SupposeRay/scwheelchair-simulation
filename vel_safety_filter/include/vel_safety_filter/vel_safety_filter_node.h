// ROS
#include <ros/ros.h>
// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include<sensor_msgs/point_cloud_conversion.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>
// C++
#include <stdlib.h>
#include <math.h>
// Packages
#include <scat_libs/obst_utils.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/base_utils.h>
#include <scat_libs/nav_utils.h>
#include <scat_libs/rosmsg.h>
#include <scat_msgs/SetMode.h>
#include <scat_libs/diff_drive.h>
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>

namespace vel_safety_filter
{

class VelFilterNode
{
public:
  // Constructor
  VelFilterNode(ros::NodeHandle &node_handle);
  // Destructor
  virtual ~VelFilterNode();

private:
  //// Node methods ////
  bool readParameters();
  bool readBotParameters();
  void publishResults(geometry_msgs::Twist);
  void controlCmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd);
  void userCmdCallback(const geometry_msgs::Twist::ConstPtr &msg_cmd);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);
  void velCallback(const geometry_msgs::Twist::ConstPtr &msg_vel);
  void timerCallback(const ros::TimerEvent &);
  bool serviceCallback(scat_msgs::SetModeRequest &request,
                       scat_msgs::SetModeResponse &response);
                       
  // Only one of the below callbacks will be used, depending on which value is set for obst_type: "scan", "obstmsg", or "cloud"
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan);
  void obstCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_obst);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_cloud);



  //// Algorithm methods ////
  // Calls all the checks to be performed on the velocity command, i.e. checkAcceleration, checkVelocity and checkCollision.
  void processCmd(const geometry_msgs::Twist &msg_cmd);
  // Predicts future position of robot footprint based on velocity command cmd_vel, and checks whether this is in collision with current LaserScan measurements.
  geometry_msgs::Twist checkCollision(geometry_msgs::Twist cmd_vel);

  //// State containers ////
  // Command velocity
  geometry_msgs::Twist cmd_vel_;
  //! laserscan data of the environment
  sensor_msgs::LaserScan scan_;
  //! point cloud of scan message
  sensor_msgs::PointCloud point_cloud_;

  //! path
  nav_msgs::Path path_;
  // footprint marker msg
  visualization_msgs::Marker footprint_marker_;

  //// Variables ////
  // time variables
  ros::Time t_now_, t_prev_;
  //! boolean indicating whether node has an updated scan
  bool scan_received_ = false;
  bool cloud_received_ = false;
  bool obst_received_ = false;
  //! boolean indicating whether node has an updated velocity
  bool vel_received_ = false;
  //! boolean indicating whether node should apply acceleration limts
  bool use_vel_filter_ = true;
  bool use_acc_filter_ = true;
  //! boolean indicating whether node should apply collision check
  bool use_col_filter_ = true;
  // current velocities from odometry
  float v_agent_ = 0, w_agent_ = 0;
  geometry_msgs::Twist vel_agent_;
  // Obstacles as a vector of vectors of points. Each vector of points describes a single obstacle.
  std::vector<std::vector<geometry_msgs::Point>> obstacles_pts_;

  //// Parameters ////
  //! base frame id
  std::string base_frame_id_ = "base_footprint";
  //! fixed frame id
  std::string fixed_frame_id_ = "odom";
  // mode. This is used to determine which source is used for the control commands.
  // If mode is autonomous or shared, commands from /control/cmd_vel topic are passed on.
  // If mode is manual, commands from /user/cmd_vel are passed on.
  std::string mode_ = "autonomous";
  //! string determining whether using "footprint" collision method or "radius" collision method
  std::string collision_method_ = "footprint";
  // vel type parameter setting
  std::string vel_type_ = "odom";
  //! parameter determining how far from obstacle the safety filter limits the velocity
  float obst_inflation_ = 0.2;      // [m]
  float footprint_inflation_ = 0.2; // 0.1 = 10%
  //! parameter determining angle range which is check for collision
  float angle_res_ = 0.2; // [deg]
  //! time after which the safety filter automatically publishes a 0 cmd_vel, if no other commands are received.
  float time_out_ = 1.0;
  // time inerval for path prediction
  float sampling_time_ = 0.1;
  // Inflation factr for velocity. As the robot speeds up, the footprint is enlarged by this factor (0.2 = 20%). This results in slowing down before going through narrow passageways.
  float vel_inflation_ = 0.2;
  // parameter determing which source is used for collision checking. can be "cloud", "scan", or "obstmsg"
  std::string obst_type_ = "scan";
  // Parameter indicating the range within which the velocity commands are checked for collsion
  float obst_range_ = 10.0;
  // threshold to avoid taking the floor into account. Any points in the point cloud, with height above this threshold, will be checked for obstacle avoidance in the collision check. 
  float height_threshold_ = 0.3;
  //// Robot specific parameters
  //! Maximum linear and angular velocity
  float v_max_ = 0.8, v_min_ = 0.35, v_max_neg_ = -0.7, w_max_ = 1.8, w_min_ = 1.4; // [m/s] and [deg/s]
  // algorithm activation thresholds
  float v_threshold_ = 0.1, w_threshold_ = 0.1;
  // Maximum linear and angular acceleration
  float v_acc_ = 0.6, w_acc_ = 1.5; // m/s^2 and deg/s^2
  // Maximum brake accelarations, or 'Minimum' linear and angular acceleration
  float v_dec_ = 0.6, w_dec_ = 1.5; // m/s^2 and deg/s^2
  //! robotradius
  float robot_radius_ = 0.5;
  // wheel gap used for differential drive calculations
  float base_width_ = 0.5;
  // largest distance from center of footprint to corner of footprint
  float L_max_ = 0;
  // Input parameters for footprint.
  std::vector<float> footprint_coords_;
  //! footprint data
  std::vector<geometry_msgs::Point> footprint_;

  //// ROS objects
  ros::NodeHandle &node_handle_;

  ros::Subscriber usercmd_subscriber_, ctrlcmd_subscriber_,
      obst_subscriber_, odom_subscriber_, vel_subscriber_;

  ros::Publisher cmd_publisher_, shape_publisher_, path_publisher_;

  tf::TransformListener tf_listener_;
  laser_geometry::LaserProjection projector_;
  ros::ServiceServer mode_service_;
  ros::Timer timer_;

  // Vel Utils object to calculate the limited velocity
  diff_drive::VelUtils vel_utils_;
};

} // namespace vel_safety_filter
