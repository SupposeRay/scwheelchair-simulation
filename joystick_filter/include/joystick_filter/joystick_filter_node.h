// ROS
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
// Messages
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <deque>
// #include <Eigen/Dense>
#include <numeric>
// Package
#include <scat_libs/base_utils.h>
#include <scat_libs/rosmsg.h>

namespace joystick_filter_node
{

class FilterNode
{
public:
  FilterNode(ros::NodeHandle &node_handle);

  virtual ~FilterNode();

private:
  // Node methods
  bool readParameters();
  void initializeMatrices();
  void resetMatrices();
  void resetBuffer();
  void userinputCallback(const std_msgs::Float32MultiArray &msg_userinput);
  void timerCallback(const ros::TimerEvent &);
  void publishResults(const std_msgs::Float32MultiArray &userinput);

  // Algorithm methods
  void deadZone(std_msgs::Float32MultiArray &userinput);
  void kalmanFilter(std_msgs::Float32MultiArray &userinput);
  void medianFilter(std_msgs::Float32MultiArray &userinput);
  void expControlFunction(std_msgs::Float32MultiArray &userinput);
  void kalmanPredict();
  void kalmanUpdate();

  /////// Node parameters ///////
  // Publish period
  double publish_frequency_ = 10.0;
  // deadzone parameters
  float deadzone_ = 0.25;
  // velocity scaling parameters
  float k_v_ = 0.7;
  float k_w_ = 1.5;
  // Decides whether joy data moves through Kalmann filter
  bool use_kalmannfilter_ = true;
  // Decides whether joy data moves through moving average filter
  bool use_medianfilter_ = false;
  // decides whether to make kalman filter adaptive
  bool adaptive_kalman_ = false;
  // whether to display variance matrices
  bool display_variance_matrices_ = false;
  // exponent used to ramp up the joystick output value farther off-center
  float control_exp_ = 1.0;

  /////// Kalman parameters ///////
  // inputs to measurement covariance matrix
  float cov_x_ = 0.1, cov_y_ = 0.1, cov_vx_ = 0.1, cov_vy_ = 0.1;
  // inputs to state transtion matrix
  float Hx_ = 5.0, Hy_ = 5.0;
  // inputs to initial uncertainty matrix
  float P0_ = 100;
  // Adaptive covariance forgetting factor
  float alpha_ = 0.8;
  // inputs to Process covariance matrix
  float Qx_ = 0.1, Qy_ = 0.1;
  // direct user joystick input in x & y direction, scaled to -1:1.
  float ux_ = 0, uy_ = 0, vx_ = 0, vy_ = 0;

  ///////// Median Filter Parameters ////////
  int buffer_size_ = 5;

  ///////// Variables /////////
  // time of arrival of new measurement
  ros::Time t_, t_prev_;
  // time since last measurement
  double dt_;
  
  ///////// Median filter variables /////////
  std::deque<float> buffer_x_;
  std::deque<float> buffer_y_;
  float average_x_ = 0;
  float average_y_ = 0;

  ///////// Kalman filter variables /////////
  // state vector
  Eigen::VectorXf X_ = Eigen::VectorXf(4, 1);
  // state covariance matrix
  Eigen::MatrixXf P_ = Eigen::MatrixXf(4, 4);
  // state transition matrix
  Eigen::MatrixXf F_ = Eigen::MatrixXf(4, 4);
  // measurement matrix
  Eigen::MatrixXf H_ = Eigen::MatrixXf(4, 4);
  // measurement covariance matrix
  Eigen::MatrixXf R_ = Eigen::MatrixXf(4, 4);
  // process covariance matrix
  Eigen::MatrixXf Q_ = Eigen::MatrixXf(4, 4);
  // Measurement vector
  Eigen::VectorXf Z_ = Eigen::VectorXf(4, 1);
  // Measurement residual vector
  Eigen::VectorXf Y_ = Eigen::VectorXf(4, 1);
  // Measurement residual vector
  Eigen::VectorXf eps_ = Eigen::VectorXf(4, 1);
  // Measurement noise matrix
  Eigen::MatrixXf S_ = Eigen::MatrixXf(4, 4);
  // Kalman gain matrix
  Eigen::MatrixXf K_ = Eigen::MatrixXf(4, 4);
  // Identity matrix
  Eigen::MatrixXf I_ = Eigen::MatrixXf::Identity(4, 4);

  // adaptive measurement covariance matrix
  Eigen::MatrixXf Ra_ = Eigen::MatrixXf(4, 4);
  // adaptive process covariance matrix
  Eigen::MatrixXf Qa_ = Eigen::MatrixXf(4, 4);

  // ROS objects
  ros::NodeHandle &node_handle_;
  ros::Subscriber userinput_subscriber_;
  ros::Publisher userinput_publisher_, usercmd_publisher_, performance_publisher_;
  ros::Timer timer_;
  
};

} // namespace joystick_filter_node
