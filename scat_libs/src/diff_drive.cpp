#include "scat_libs/diff_drive.h"

namespace diff_drive 
{

// takeStep takes a single time step, calculating the next position, using linear velocity v, angular velocity w, sampling time dt
void takeStep(geometry_msgs::Pose2D& pose, const float& v, const float& w, const float& dt)
{
  pose.x += v * cos(pose.theta) * dt;
  pose.y += v * sin(pose.theta) * dt;
  pose.theta += w * dt;
}

// The VelUtils object is used to limit velocity and acceleration to certain limits.
// The limits can be provided (usually once) by the setParams function.
// Then the limitVelocity or limitAccleration function can be called.
// Two overloaded methods are implemented for each function, with the same functionality but different interface (one using floats and another using ROS message types.)

// Constructor
VelUtils::VelUtils() {}

// Destructor
VelUtils::~VelUtils() {}

void VelUtils::setParams(
    const float &v_min, const float &v_max,
    const float &v_max_neg,
    const float &w_min, const float &w_max,
    const float &v_acc, const float &v_dec,
    const float &w_acc, const float &w_dec,
    const float &v_threshold, const float &w_threshold,
    const float &sampling_time, const float &base_width)
{
  v_min_ = v_min;
  v_max_ = v_max;
  v_max_neg_ = v_max_neg;
  w_min_ = w_min;
  w_max_ = w_max;
  v_acc_ = v_acc;
  v_dec_ = v_dec;
  w_acc_ = w_acc;
  w_dec_ = w_dec;
  v_threshold_ = v_threshold;
  w_threshold_ = w_threshold;
  dt_ = sampling_time;
  base_width_ = base_width;
}

geometry_msgs::Twist VelUtils::limitAcceleration(const geometry_msgs::Twist &cmd_vel,
                                                 const geometry_msgs::Twist &vel_agent)
{
  float v = cmd_vel.linear.x;
  float w = cmd_vel.angular.z;
  float v_agent = cmd_vel.linear.x;
  float w_agent = cmd_vel.angular.z;
  limitAcceleration(v, w, v_agent, w_agent);
  return rosmsg::makeTwist(v, w);
}

void VelUtils::limitAcceleration(float &v, float &w, const float &v_agent, const float &w_agent)
{
  if (v > 0.0f)
    v = std::max(std::max(std::min(v, v_agent + v_acc_ * dt_), v_agent - v_dec_ * dt_), 0.0f);
  else if (v < 0.0f)
    v = std::min(std::min(std::max(v, v_agent - v_acc_ * dt_), v_agent + v_dec_ * dt_), 0.0f);

  if (w > 0.0f)
    w = std::max(std::max(std::min(w, w_agent + w_acc_ * dt_), w_agent - w_dec_ * dt_), 0.0f);
  else if (w < 0.0f)
    w = std::min(std::min(std::max(w, w_agent - w_acc_ * dt_), w_agent + w_dec_ * dt_), 0.0f);
}

// Limits the velocity according to the parameter set by setParameters
// Returns the limited velocity in the reference
geometry_msgs::Twist VelUtils::limitVelocity(const geometry_msgs::Twist &cmd_vel)
{
  float v = cmd_vel.linear.x;
  float w = cmd_vel.angular.z;
  limitVelocity(v, w);
  return rosmsg::makeTwist(v, w);
}

// Limits the velocity according to the parameter set by setParameters
// Returns the limited velocity as a Twist
void VelUtils::limitVelocity(float &v, float &w)
{
  // Limit velocity command to within velocity & acceleration limits
  if (fabs(v) >= v_threshold_ || fabs(w) >= w_threshold_)
  {
    // Map linear and angular velocity to individual wheel velocities
    float v_left = v - w * base_width_ / 2;
    float v_right = v + w * base_width_ / 2;
    // Calculated squared metric of combined velocities
    float v_sq = sqrt(pow(v_left, 2) + pow(v_right, 2));
    if (v_sq < sqrt(2) * v_min_)
    {
      // Scale v_left & v_right according to law of similar triangles. A' = A*C'/C
      v_left = v_left * sqrt(2) * v_min_ / v_sq;
      v_right = v_right * sqrt(2) * v_min_ / v_sq;
      w = (v_right - v_left) / base_width_;
      v = (v_left + v_right) / 2;
    }
  }
  // Velocity outer limits
  if (fabs(w) > w_max_)
    w = base_utils::sign(w) * w_max_;
  if (v < v_max_neg_)
    v = v_max_neg_;
  else if (v > v_max_)
    v = v_max_;
}

// Store the predicted future path of a differential drive robot in a nav_msgs::Path message, based on current coordinates, velocity (of type double), number of samples and time difference. A header has to be supplied. 
nav_msgs::Path makePath(double x, double y, double theta, const double& v, const double& w, const int& samples, const double& dt, const std_msgs::Header& header)
{
  nav_msgs::Path path;
  path.header = header;
  for (int t = 0; t < samples; t++)
  {
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;
    theta += w * dt;
    path.poses.push_back(rosmsg::makePoseStamped(x, y, theta, header));
  }
  return path;
}

// Store the predicted future path of a differential drive robot in a nav_msgs::Path message, based on current coordinates, velocity (of type twist), number of samples and time difference. A header has to be supplied. 
nav_msgs::Path makePath(double x, double y, double theta, const geometry_msgs::Twist& twist, const int& samples, const double& dt, const std_msgs::Header& header)
{
  nav_msgs::Path path;
  path.header = header;  
  for (int t = 0; t < samples; t++)
  {
    x += twist.linear.x * cos(theta) * dt;
    y += twist.linear.x * sin(theta) * dt;
    theta += twist.angular.z * dt;
    path.poses.push_back(rosmsg::makePoseStamped(x, y, theta, header));
  }
  return path;
}

// Store the predicted future path of a differential drive robot in a vector of 2d poses, based on current velocity (of type double), 
// in the local coordinate frame so no initial coordinates (starting point x = 0, y = 0, theta = 0)
std::vector<geometry_msgs::Pose2D> makePath(const double& v, const double& w, const int& samples, const double& dt)
{
  std::vector<geometry_msgs::Pose2D> path;
  double x = 0, y = 0, theta = 0;
  for (int t = 0; t < samples; t++)
  {
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;
    theta += w * dt;    
    path.push_back(rosmsg::makePose2D(x, y, theta));
  }
  return path;
}

// Store the predicted future path of a differential drive robot in a vector of 2d poses, based on current velocity (of type twist), 
// in the local coordinate frame so no initial coordinates (starting point x = 0, y = 0, theta = 0)
std::vector<geometry_msgs::Pose2D> makePath(const geometry_msgs::Twist& twist, const int& samples, const double& dt)
{
  std::vector<geometry_msgs::Pose2D> path;
  double x = 0, y = 0, theta = 0;
  for (int t = 0; t < samples; t++)
  {
    x += twist.linear.x * cos(theta) * dt;
    y += twist.linear.x * sin(theta) * dt;
    theta += twist.angular.z * dt;
    path.push_back(rosmsg::makePose2D(x, y, theta));
  }
  return path;
}

// Store the predicted future path of a differential drive robot in a nav_msgs::Path message, based on current velocity (of type double), 
// in the local coordinate frame so no initial coordinates (starting point x = 0, y = 0, theta = 0)
nav_msgs::Path makePath(const double& v, const double& w, const int& samples, const double& dt, const std_msgs::Header& header)
{
  double x = 0, y = 0, theta = 0;
  nav_msgs::Path path;
  path.header = header;
  for (int t = 0; t < samples; t++)
  {
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;
    theta += w * dt;
    path.poses.push_back(rosmsg::makePoseStamped(x, y, theta, header));
  }
  return path;
}

// Store the predicted future path of a differential drive robot in a nav_msgs::Path message, based on current velocity (of type twist), 
// in the local coordinate frame so no initial coordinates (starting point x = 0, y = 0, theta = 0)
nav_msgs::Path makePath(const geometry_msgs::Twist& twist, const int& samples, const double& dt, const std_msgs::Header& header)
{
  double x = 0, y = 0, theta = 0;
  nav_msgs::Path path;
  path.header = header;  
  for (int t = 0; t < samples; t++)
  {
    x += twist.linear.x * cos(theta) * dt;
    y += twist.linear.x * sin(theta) * dt;
    theta += twist.angular.z * dt;
    path.poses.push_back(rosmsg::makePoseStamped(x, y, theta, header));
  }
  return path;
}



} // / namespace diff_drive 
