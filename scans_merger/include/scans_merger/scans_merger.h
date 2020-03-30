#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

namespace scans_merger
{

class ScansMerger
{
public:
  ScansMerger(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~ScansMerger();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr front_scan);
  void rearScanCallback(const sensor_msgs::LaserScan::ConstPtr rear_scan);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  void publishMessages();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;

  ros::Subscriber front_scan_sub_;
  ros::Subscriber rear_scan_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher pcl_pub_;

  tf::TransformListener tf_ls_;
  laser_geometry::LaserProjection projector_;

  bool front_scan_received_;
  bool rear_scan_received_;
  bool front_scan_error_;
  bool rear_scan_error_;

  sensor_msgs::PointCloud front_pcl_;
  sensor_msgs::PointCloud rear_pcl_;

  // Parameters
  bool p_active_;
  bool p_publish_scan_;
  bool p_publish_pcl_;

  int p_ranges_num_;

  double p_min_scanner_range_;
  double p_max_scanner_range_;
  double p_min_x_range_;
  double p_max_x_range_;
  double p_min_y_range_;
  double p_max_y_range_;

  std::string p_fixed_frame_id_;
  std::string p_target_frame_id_;
};

} // namespace scans_merger
