#include "scat_libs/rosmsg.h"

// Simple utilities to  make ROS message types in a single line. 
// Just provide all the data in the command, e.g.: 
// geometry_msgs::Point new_point = makePoint(x,y);
// All methods make a new copy of the type, and return this, such that you can directly use it for other function calls. 
// More convoluted example (saving 6 lines of code): 
// ros_publisher_.publish(rosmsg::makePoseStamped(x, y, theta, rosmsg::makeHeader(ros::Time::now(),frame_id)));  // publishes current position of the robot x,y,theta, with current time stamp and specified frame_id.
// Functionality is currently heavily geared towards 2D robots. 

namespace rosmsg 
{

std_msgs::Header makeHeader(const std::string& frame_id, const ros::Time& stamp)
{
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = stamp;
  return header;
}

std_msgs::Float32MultiArray makeFloat32MultiArray(const std::vector<float>& data)
{
  std_msgs::Float32MultiArray array;
  for (const float& x : data)
    array.data.push_back(x);
  return array;
}

std_msgs::Float64MultiArray makeFloat64MultiArray(const std::vector<double>& data)
{
  std_msgs::Float64MultiArray array;
  for (const double& x : data)
    array.data.push_back(x);
  return array;
}

geometry_msgs::Twist makeTwist(const double& v, const double& w)
{
  geometry_msgs::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  return twist;
}

geometry_msgs::Point makePoint(const double& x, const double& y)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  return point;
}


geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::Point32 makePoint32(const double& x, const double& y)
{
  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  return point;
}

geometry_msgs::Point32 makePoint32(double x, double y, double z)
{
  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::PointStamped makePointStamped(const double& x, const double& y, const std_msgs::Header& header)
{
  geometry_msgs::PointStamped point;
  point.point.x = x;
  point.point.y = y;
  point.header = header;
  return point;
}

geometry_msgs::Pose makePose(const double& x, const double& y, const double& theta)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  return pose;
}

geometry_msgs::Pose2D makePose2D(const double& x, const double& y, const double& theta)
{
  geometry_msgs::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

geometry_msgs::PoseStamped makePoseStamped(const double& x, const double& y, const double& theta, const std_msgs::Header& header)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  pose.header = header;
  return pose;
}

std::vector<geometry_msgs::Point> makeMarkerPoints(const std::vector<geometry_msgs::Point>& points)
{
  std::vector<geometry_msgs::Point> markers;
  markers.push_back(points.back());
  markers.push_back(points[0]);
  for (int i = 1; i < points.size(); i++)
  {
      markers.push_back(points[i-1]);
      markers.push_back(points[i]);
  }
  return markers;
}

nav_msgs::Odometry makeOdometry(const std_msgs::Header& header, const double& x, const double& y, const double& theta, const double& v, const double& w)
{
  nav_msgs::Odometry odom;
  odom.header = header;
  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = w;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  odom.pose.covariance[0] = -1;
  odom.twist.covariance[0] = -1;
  return odom;
}

nav_msgs::Odometry makeOdometry(const std_msgs::Header &header, 
const double &x, const double &y, const double &theta, 
const double &v, const double &w,
std::vector<double> CovOdom, std::vector<double> CovVel)
{
  nav_msgs::Odometry odom;
  odom.header = header;
  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = w;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  for (int i = 0; i < 36; i++)
  {
    odom.pose.covariance[i] = CovOdom[i];
    odom.twist.covariance[i] = CovVel[i];
  }

  return odom;
}

sensor_msgs::Imu makeIMU(const std_msgs::Header& header, const double& roll, const double& pitch, const double& yaw, 
const double& GyroX, const double& GyroY, const double& GyroZ, 
const double& AccX, const double& AccY, const double& AccZ)
{
  sensor_msgs::Imu imu; 
  imu.header = header;
  imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  imu.angular_velocity.x = GyroX;
  imu.angular_velocity.y = GyroY;
  imu.angular_velocity.z = GyroZ;
  imu.linear_acceleration.x = AccX;
  imu.linear_acceleration.y = AccY;
  imu.linear_acceleration.z = AccZ;
  imu.orientation_covariance[0] = -1;
  imu.orientation_covariance[1] = -1;
  imu.linear_acceleration_covariance[0] = -1;
  imu.linear_acceleration_covariance[1] = -1;
  imu.angular_velocity_covariance[0] = -1;
  imu.angular_velocity_covariance[1] = -1;
  return imu;
}

sensor_msgs::Imu makeIMU(const std_msgs::Header& header, const double& roll, const double& pitch, const double& yaw, 
const double& GyroX, const double& GyroY, const double& GyroZ, 
const double& AccX, const double& AccY, const double& AccZ,
std::vector<double> CovOrient, std::vector<double> CovGyro, std::vector<double> CovAcc)
{
  sensor_msgs::Imu imu; 
  imu.header = header;
  imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  imu.angular_velocity.x = GyroX;
  imu.angular_velocity.y = GyroY;
  imu.angular_velocity.z = GyroZ;
  imu.linear_acceleration.x = AccX;
  imu.linear_acceleration.y = AccY;
  imu.linear_acceleration.z = AccZ;
  for (int i = 0; i < 9; i++)
  {
    imu.orientation_covariance[i] = CovOrient[i];
    imu.angular_velocity_covariance[i] = CovGyro[i];
    imu.linear_acceleration_covariance[i] = CovAcc[i];
  }

  return imu;
}

// converts path from a vector of 2D poses, to the standard ROS implementation of nav_msgs::Path
nav_msgs::Path convertPath(const std::vector<geometry_msgs::Pose2D>& path, const std_msgs::Header& header)
{
  nav_msgs::Path new_path;
  new_path.header = header;  
  for (const geometry_msgs::Pose2D& pose : path)
  {
    new_path.poses.push_back(makePoseStamped(pose.x, pose.y, pose.theta, header));
  }
  return new_path;
}


} // / namespace rosmsg 