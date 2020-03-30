#ifndef SCAT_ROSMSG
#define SCAT_ROSMSG
// ROS
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
// C++
#include <stdlib.h>
#include <math.h>

// Simple utilities to  make ROS message types in a single line.
// Just provide all the data in the command, e.g.:
// geometry_msgs::Point new_point = makePoint(x,y);
// All methods make a new copy of the type, and return this, such that you can directly use it for other function calls.
// More convoluted example (saving 6 lines of code):
// ros_publisher_.publish(rosmsg::makePoseStamped(x, y, theta, rosmsg::makeHeader(ros::Time::now(),frame_id)));  // publishes current position of the robot x,y,theta, with current time stamp and specified frame_id.
// Functionality is currently heavily geared towards 2D robots.

namespace rosmsg
{
std_msgs::Header makeHeader(const std::string &frame_id, const ros::Time &stamp);
std_msgs::Float32MultiArray makeFloat32MultiArray(const std::vector<float> &data);
std_msgs::Float64MultiArray makeFloat64MultiArray(const std::vector<double> &data);
geometry_msgs::Twist makeTwist(const double &v, const double &w);
geometry_msgs::Point makePoint(const double &x, const double &y);
geometry_msgs::Point makePoint(double x, double y, double z);
geometry_msgs::Point32 makePoint32(double x, double y, double z);
geometry_msgs::Point32 makePoint32(const double &x, const double &y);
geometry_msgs::PointStamped makePointStamped(const double &x, const double &y, const std_msgs::Header &header);
geometry_msgs::Pose makePose(const double &x, const double &y, const double &theta);
geometry_msgs::Pose2D makePose2D(const double &x, const double &y, const double &theta);
geometry_msgs::PoseStamped makePoseStamped(const double &x, const double &y, const double &theta, const std_msgs::Header &header);
std::vector<geometry_msgs::Point> makeMarkerPoints(const std::vector<geometry_msgs::Point> &points);
nav_msgs::Odometry makeOdometry(const std_msgs::Header &header, 
const double &x, const double &y, const double &theta, 
const double &v, const double &w);
nav_msgs::Odometry makeOdometry(const std_msgs::Header &header, 
const double &x, const double &y, const double &theta, 
const double &v, const double &w,
std::vector<double> CovOdom, std::vector<double> CovVel);
sensor_msgs::Imu makeIMU(const std_msgs::Header &header, const double &roll, const double &pitch, const double &yaw,
                         const double &GyroX, const double &GyroY, const double &GyroZ,
                         const double &AccX, const double &AccY, const double &AccZ);
sensor_msgs::Imu makeIMU(const std_msgs::Header &header, const double &roll, const double &pitch, const double &yaw,
                                       const double &GyroX, const double &GyroY, const double &GyroZ,
                                       const double &AccX, const double &AccY, const double &AccZ,
                                       std::vector<double> CovOrient, std::vector<double> CovGyro, std::vector<double> CovAcc);
nav_msgs::Path convertPath(const std::vector<geometry_msgs::Pose2D> &path, const std_msgs::Header &header);
} // namespace rosmsg

#endif