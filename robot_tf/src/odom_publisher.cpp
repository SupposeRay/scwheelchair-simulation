
#include <ros/ros.h>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>                       // outgoing message type
#include <geometry_msgs/Pose2D.h>                    // outgoing message type
#include <geometry_msgs/PoseWithCovarianceStamped.h> // outgoing message type
#include <geometry_msgs/Pose.h>                      // outgoing message type
#include <geometry_msgs/Twist.h>                     // outgoing message type
#include <stdlib.h>
#include <math.h>
#include <chrono>

geometry_msgs::Twist vel_scan, cmd_vel_;
nav_msgs::Odometry odom_scan, odom_cmd;
geometry_msgs::Pose2D prev_pose;

double sign_vel = 0;
double theta_cmd = 0;
std::chrono::system_clock::time_point time_now_cmd, time_prev_cmd, time_now_scan, time_prev_scan;

double wrapMax(double x, double max)
{
  return fmod(max + fmod(x, max), max);
}
double wrapMinMax(double x, double min, double max)
{
  return min + wrapMax(x - min, max - min);
}

void pose2DCallback(const geometry_msgs::Pose2D msg_pose)
{
  time_now_scan = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = time_now_scan - time_prev_scan;
  double dt = elapsed_time.count();
  double dtheta = wrapMinMax(msg_pose.theta - atan2(msg_pose.y - prev_pose.y, msg_pose.x - prev_pose.x), -M_PI, M_PI);
  if (fabs(dtheta) > M_PI / 2)
    sign_vel = -1;
  else
    sign_vel = 1;

  vel_scan.linear.x = sign_vel * pow(pow(msg_pose.x - prev_pose.x, 2) + pow(msg_pose.y - prev_pose.y, 2), 0.5) / dt;
  vel_scan.angular.z = (msg_pose.theta - prev_pose.theta) / dt;

  odom_scan.twist.twist = vel_scan;
  odom_scan.pose.pose.position.x = msg_pose.x;
  odom_scan.pose.pose.position.y = msg_pose.y;
  odom_scan.pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg_pose.theta);
  odom_scan.pose.covariance[0] = -1;
  odom_scan.twist.covariance[0] = -1;

  prev_pose = msg_pose;
  time_prev_scan = time_now_scan;
}

void cmdCallback(const geometry_msgs::Twist msg_cmd)
{

  time_now_cmd = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = time_now_cmd - time_prev_cmd;
  double dt = elapsed_time.count();
  odom_cmd.twist.twist = msg_cmd;
  theta_cmd += cmd_vel_.angular.z * dt;
  odom_cmd.pose.pose.position.x += cos(theta_cmd) * cmd_vel_.linear.x * dt;
  odom_cmd.pose.pose.position.y += sin(theta_cmd) * cmd_vel_.linear.x * dt;
  odom_cmd.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_cmd);
  odom_cmd.pose.covariance[0] = -1;
  odom_cmd.twist.covariance[0] = -1;
  cmd_vel_ = msg_cmd;
  time_prev_cmd = time_now_cmd;
}

int main(int argc, char **argv)
{
  ///// Initialize Node
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle node_handle;

  // initialize global variables
  prev_pose.x = 0;
  prev_pose.y = 0;
  prev_pose.theta = 0;
  cmd_vel_.linear.x = 0;
  cmd_vel_.angular.z = 0;
  time_prev_cmd = std::chrono::system_clock::now();
  time_now_scan = std::chrono::system_clock::now();

  ///// Subscribers /////
  ros::Subscriber userinput_subscriber = node_handle.subscribe("/pose2D", 1, pose2DCallback);
  ros::Subscriber cmd_subscriber = node_handle.subscribe("/cmd_vel", 1, cmdCallback);

  ///// Publishers /////
  ros::Publisher odom_scan_publisher = node_handle.advertise<nav_msgs::Odometry>("/odom_scan", 1);
  ros::Publisher vel_scan_publisher = node_handle.advertise<geometry_msgs::Twist>("/vel_scan", 1);
  ros::Publisher odom_cmd_publisher = node_handle.advertise<nav_msgs::Odometry>("/odom_cmd", 1);

  ros::Rate loop_rate(50);
  ros::spinOnce();

  while (ros::ok())
  {
    odom_scan_publisher.publish(odom_scan);
    vel_scan_publisher.publish(vel_scan);
    odom_cmd_publisher.publish(odom_cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
