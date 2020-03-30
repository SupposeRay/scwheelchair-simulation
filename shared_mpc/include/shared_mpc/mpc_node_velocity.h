// ROS
#include <ros/ros.h>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>

// C++
#include <stdlib.h>
#include <math.h>
#include <thread>
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <typeinfo>
// Package
#include <sparse_voronoi/voronoi_algorithm.h>
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>
#include <scat_libs/base_utils.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/diff_drive.h>
#include "shared_mpc/fg_eval_diff_drive.h"
#include <scat_libs/obst_utils.h>

// Package
namespace shared_mpc
{
using CppAD::AD;

class MPCNode
{
public:
  // Constructor
  MPCNode(ros::NodeHandle &node_handle);
  // Destructor
  virtual ~MPCNode();

  typedef CPPAD_TESTVECTOR(double) Dvector;

  bool use_voronoi_ = true;

private:
  //// Node methods
  bool readParameters();
  bool readVoronoiParameters();
  bool readIpoptParameters();
  bool readBotParameters();

  void initialize();
  void userinputCallback(const geometry_msgs::Twist::ConstPtr &msg_user);
  void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_position);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);
  void laserObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects);
  void camObjectCallback(const scat_msgs::EnvObjectList::ConstPtr &msg_objects);
  void perfCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_perf);
  void publishResults();
  bool serviceCallback(std_srvs::SetBoolRequest &request,
                       std_srvs::SetBoolResponse &response);

  //// MPC methods
  void solveMPC(std::vector<double> &U0,
                std::vector<double> &U_ref,
                std::vector<CppAD::vector<CppAD::vector<AD<double>>>> &obstacles);

  void processObstacles();
  void convertObstacleToCppad(const scat_msgs::EnvObject &object);
  void addPolygonMPC(std::vector<float> polygon_params);
  void addLineMPC(float x1, float y1, float x2, float y2);
  CppAD::vector<AD<double>> addEllipseMPC(float x1, float y1, float x2, float y2);
  CppAD::vector<AD<double>> addCircleMPC(float x, float y, float r, float vx, float vy);

  void loadWarmStartPath(double v, double w);
  void loadPath(nav_msgs::Path path, double v, double w);
  void loadPath(std::vector<geometry_msgs::Pose2D> path, double v, double w);

  //// State Containers ////
  // Agent position
  geometry_msgs::Pose2D global_pose2D_;
  // Velocity commands calculated by MPC
  geometry_msgs::Twist cmd_mpc_;
  // calculated paths
  nav_msgs::Path user_path_, mpc_path_;
  // Obstacle message header
  std_msgs::Header base_header_;
  // Obstacles used in voronoi / mpc calculation
  scat_msgs::EnvObjectList cam_object_msg_, laser_object_msg_;
  // place to return solution MPC
  CppAD::ipopt::solve_result<Dvector> solution_;

  //// Variables ////
  // Setting which determines whether voronoi prediction is used
  bool use_adaptive_ = false;
  bool use_corner_constraints_ = false;
  bool node_active_ = false;
  // Booleans for message reception
  bool pos_received_ = false;
  bool cmd_received_ = false;
  bool odom_received_ = false;
  bool laser_obstacles_received_ = false;
  bool cam_obstacles_received_ = false;
  // IPOPT succes result
  bool succes_ = false;
  // State variables
  double x_agent_ = 0, y_agent_ = 0, theta_agent_ = 0;
  double v_user_ = 0, w_user_ = 0;
  double v_agent_ = 0, w_agent_ = 0;
  // Timing variables
  ros::Time t_cmd_now_, t_cmd_prev_;
  ros::Time t_obst_now_, t_obst_prev_;
  std::chrono::system_clock::time_point start_time_, end_time_;
  // obstacle data in CppAD format (Automatic Differentiation toolbox, required for MPC)
  CppAD::vector<CppAD::vector<AD<double>>> ellipses_, circles_;
  // list of obstacles in vector<point> format, transformed into local coordinate frame. Required for Voronoi diagram.
  std::vector<std::vector<geometry_msgs::Point>> obstacles_pts_;
  // indices of states and inputs
  size_t ind_x_, ind_y_, ind_theta_, ind_v_, ind_w_, ind_circles_, ind_ellipses_;
  // Number of variables (x,y,theta,v,w) and number of constraints
  size_t n_vars_, n_constraints_;
  // Vector containing variables (states and inputs) along the prediction horizon. 
  // It is shaped as: vars_ = [x1 ... xT, y1 ... yT, theta1 ... thetaT, v1 ... vT, w1 ... wT] where x1 ... xT signifies the x-values for each step along the trajectory, from t =1 to the prediction horizon t=T
  Dvector vars_;
  // Bounds on variables
  Dvector vars_lowerbound_, vars_upperbound_;
  // Bounds on constraints
  // Dvector constraints_lowerbound_, constraints_upperbound_;
  // constraints size variables
  size_t n_states_ = 3;
  size_t n_inputs_ = 2;
  size_t n_circles_ = 0;
  size_t n_ellipses_ = 0;
  // some parameter vectors
  std::vector<double> U_max_ = std::vector<double>(2, 0.0);
  std::vector<double> U_min_ = std::vector<double>(2, 0.0);
  std::vector<double> constants_ = std::vector<double>(6, 0.0);
  
  //// Parameters
  // IPOPT solver options
  std::string solver_options_;
  // Reference frame. All incoming obstacles must have this frame id. If it represents e.g. map, localization is needed.
  // All calcuations are done in this frame_id. If it represents a local frame_id (e.g. base_footprint), no localization is needed.
  std::string fixed_frame_id_ = "map", base_frame_id_ = "base_footprint";
  // Time parameters for regulating calculation frequency
  double calc_frequency_ = 10.0, obst_frequency_ = 10.0;
  // prediction horizon
  double prediction_horizon_ = 6;
  size_t Np_ = 10;
  double Np_d_ = 10;
  // time interval used for user path prediction
  double time_interval_ = 0.4;
  // Maximum path length of path constructed from voronoi tree
  double path_dist_max_ = 10.0;
  // range within which obstacles are considered
  double obst_range_ = 10.0;
  // threshold for obstacle distance at which the point is considered to be in collision
  double obst_dist_threshold_ = 0.5;
  // Cost function constants
  double Kv_ = 0.25, Kw_ = 0.0, Kh_ = 0.65, Ks_ = 0.1, Keq_ = 10, Kineq_ = 10;
  double Kv0_ = 0.2, Kw0_ = 0.0, Kh0_ = 0.6;
  double Kvr_ = 0.2, Kwr_ = 0.0, Khr_ = 0.2;
  // Robot specific parameters:
  double v_max_ = 0.7, v_min_ = 0.35, v_max_neg_ = -0.7, w_max_ = 1.8, w_min_ = 1.4;
  double v_acc_ = 0.6, v_dec_ = 0.6, w_acc_ = 1.5, w_dec_ = 1.5, v_threshold_ = 0.1, w_threshold_ = 0.1;
  double base_width_ = 0.5, robot_radius_ = 0.5;
  // positioning method, can be global or odom
  std::string pos_method_ = "global";

  //// ROS objects
  ros::NodeHandle &node_handle_;
  ros::ServiceServer activation_service_;
  // Subscribers
  ros::Subscriber position_subscriber_, userinput_subscriber_, odom_subscriber_,
      laser_obstacle_subscriber_, cam_obstacle_subscriber_, userperf_subscriber_;
  // Publishers
  ros::Publisher vorolines_publisher_, delaunaylines_publisher_,
      voropath_publisher_, mpc_path_publisher_, user_path_publisher_, cmd_publisher_;

  tf::TransformListener tf_listener_;

  // Cost function & constraints evaluation object
  shared_mpc::FG_eval fg_eval_;

  // Voronoi algorithm object
  sparse_voronoi::Voronoi_Algorithm voronoi_algorithm_;
};

} // namespace shared_mpc
