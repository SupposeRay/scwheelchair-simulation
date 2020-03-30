/*
mobile robot pose from laser
*/

#ifndef _LASER_FEATURE_ROS_H_
#define _LASER_FEATURE_ROS_H_

#include <vector>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <laserline/line_feature.h>
#include <scat_msgs/EnvObject.h>
#include <scat_msgs/EnvObjectList.h>

namespace line_feature
{

class LaserFeatureROS
{
public:
	LaserFeatureROS(ros::NodeHandle &, ros::NodeHandle &);
	//
	~LaserFeatureROS();
	//
	void startgame();

private:
	// Members
	void loadParameters();
	void publishMarkerMsg(const std::vector<gline> &, visualization_msgs::Marker &marker_msg);
	void publishLineMsg(const std::vector<gline> &m_gline, scat_msgs::EnvObjectList &object_msg);
	void compute_bearing(const sensor_msgs::LaserScan::ConstPtr &);
	void scanValues(const sensor_msgs::LaserScan::ConstPtr &);

	bool com_bearing_flag;
	bool show_lines_;
	double m_startAng;
	double m_AngInc;
	LineFeature line_feature_;

	std::string frame_id_;
	std::string scan_frame_id_;
	std::string scan_topic_;

	std::vector<gline> m_gline;
	std::vector<line> m_line;
	//ROS
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;
	ros::Subscriber scan_subscriber_;
	ros::Publisher line_publisher_;
	ros::Publisher marker_publisher_;
};

} // namespace line_feature
#endif
