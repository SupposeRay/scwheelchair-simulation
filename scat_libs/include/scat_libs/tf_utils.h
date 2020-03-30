#ifndef SCAT_TF_UTILS
#define SCAT_TF_UTILS
// ROS
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <valarray>
// Package
#include <scat_libs/base_utils.h>
#include <scat_libs/rosmsg.h>

namespace tf_utils
{
// transform based methods
geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::StampedTransform &transform);
geometry_msgs::Point32 transformPoint(const geometry_msgs::Point32 &point, const tf::StampedTransform &transform);
geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf::StampedTransform &transform);

// tf listener based methods
geometry_msgs::PoseArray transformPoseArray(const geometry_msgs::PoseArray &posearray,
                                            const std::string &target_frame,
                                            const tf::TransformListener &tf_listener);

// sensor_msgs::PointCloud transformPointCloud(const sensor_msgs::PointCloud &posearray,
// const std::string &target_frame, tf::TransformListener& tf_listener);
} // namespace tf_utils
#endif