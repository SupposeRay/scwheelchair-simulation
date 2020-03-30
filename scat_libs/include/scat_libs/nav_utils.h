#ifndef SCAT_NAV_UTILS
#define SCAT_NAV_UTILS
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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <vector>
// Package
#include <scat_libs/base_utils.h>
#include <scat_libs/rosmsg.h>
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>

namespace nav_utils
{

std::vector<geometry_msgs::Point> createPolygon2D(const std::vector<double> &coords,
                                                  const double &inflation, const int &coord_dim);
std::vector<geometry_msgs::Point> createPolygon2D(const std::vector<float> &coords,
                                                  const float &inflation, const int &coord_dim);

std::vector<geometry_msgs::Point> moveFootprint(std::vector<geometry_msgs::Point> footprint,
                                                const double &x, const double &y, const double &theta);

bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const sensor_msgs::PointCloud &pointcloud,
                                const double &x, const double &y, const double &theta);

bool checkRadiusForCollision(const double &robot_radius,
                             const sensor_msgs::LaserScan &scan,
                             const double &x, const double &y, const double &theta,
                             const double &angle_res, const double &obst_inflation);

bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const sensor_msgs::PointCloud &pointcloud,
                                const double &x, const double &y, const double &theta,
                                std::vector<geometry_msgs::Point32> &coll_points);

bool checkRadiusForCollision(const double &robot_radius,
                             const sensor_msgs::LaserScan &scan,
                             const double &x, const double &y, const double &theta,
                             const double &angle_res, const double &obst_inflation,
                             std::vector<double> &coll_angles);

bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const std::vector<std::vector<geometry_msgs::Point>> &obstacles,
                                const double &x, const double &y, const double &theta);

bool checkFootprintForCollision(std::vector<geometry_msgs::Point> footprint,
                                const std::vector<std::vector<geometry_msgs::Point>> &obstacles,
                                const double &x, const double &y, const double &theta,
                                std::vector<geometry_msgs::Point32> &coll_points);

} // namespace nav_utils
#endif
