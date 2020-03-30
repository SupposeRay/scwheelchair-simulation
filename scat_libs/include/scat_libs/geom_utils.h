#ifndef SCAT_GEOM_UTILS
#define SCAT_GEOM_UTILS
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

namespace geom_utils
{
double determinant(double v1[2], double v2[2]);
double determinant(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
void findInCenter(double a[2], double b[2], double c[2], double incenter[2]);
void findCircumCenter(double a[2], double b[2], double c[2], double circumcenter[2]);
geometry_msgs::Point findInCenter(const std::vector<geometry_msgs::Point>& pts);
geometry_msgs::Point findCircumCenter(const std::vector<geometry_msgs::Point>& pts);
geometry_msgs::Point findInCenter(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c);
geometry_msgs::Point findCircumCenter(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c);
bool doIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& q1, const geometry_msgs::Point& q2);
bool doIntersect(double p1[2], double p2[2], double q1[2], double q2[2]);
bool onSegment(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& q);
bool onSegment(double p1[2], double p2[2], double q[2]);
int orientation(const geometry_msgs::Point& p, const geometry_msgs::Point& q, const geometry_msgs::Point& r);
int orientation(double p[2], double q[2], double r[2]);
bool isOnOtherSide(double p1[2], double p2[2], double q1[2], double q2[2]);
bool isOnOtherSide(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& q1, const geometry_msgs::Point& q2);
bool isInsidePolygon(const std::vector<geometry_msgs::Point>& polygon, const geometry_msgs::Point32& point);
bool isInsidePolygon(const std::vector<geometry_msgs::Point>& polygon, const geometry_msgs::Point& point);

geometry_msgs::Point localToGlobal(const geometry_msgs::Point& point_local, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame);
geometry_msgs::Point localToGlobal(const geometry_msgs::Point& point_local, const geometry_msgs::Pose2D& local_frame);
geometry_msgs::Point32 localToGlobal(const geometry_msgs::Point32& point_local, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame);
geometry_msgs::Point32 localToGlobal(const geometry_msgs::Point32& point_local, const geometry_msgs::Pose2D& local_frame);
void localToGlobal(double &x, double &y, double &theta, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame);

geometry_msgs::Point globalToLocal(const geometry_msgs::Point& point_global, const geometry_msgs::Pose2D& local_frame);
geometry_msgs::Point32 globalToLocal(const geometry_msgs::Point32& point_global, const geometry_msgs::Pose2D& local_frame);
geometry_msgs::Point32 globalToLocal(const geometry_msgs::Point32& point_global, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame);
geometry_msgs::Point globalToLocal(const geometry_msgs::Point& point_global, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame);
void globalToLocal(double &x, double &y, double &theta, const double& x_local_frame, const double& y_local_frame, const double& theta_local_frame);

double distanceToLine(const geometry_msgs::Point &point,
                      const geometry_msgs::Point &point1,
                      const geometry_msgs::Point &point2);
double distanceToLineSegment(const geometry_msgs::Point &point,
                             const geometry_msgs::Point &point1,
                             const geometry_msgs::Point &point2);
geometry_msgs::Point findClosestPointOnInterval(const geometry_msgs::Point &point,
                                                const geometry_msgs::Point &point1,
                                                const geometry_msgs::Point &point2);
geometry_msgs::Point findClosestPoint(const geometry_msgs::Point &point,
                                      const geometry_msgs::Point &point1,
                                      const geometry_msgs::Point &point2);
bool onInterval(const geometry_msgs::Point &point,
                const geometry_msgs::Point &point1,
                const geometry_msgs::Point &point2);

geometry_msgs::Point calcClosestPointOnLine(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Pose& pose1,
                                            const geometry_msgs::Pose& pose2);
geometry_msgs::Point calcIntersection(const geometry_msgs::Pose& pose1,
                                      const geometry_msgs::Pose& pose2);

double angleDisparity(const geometry_msgs::Quaternion& quat1,
                      const geometry_msgs::Quaternion& quat2);
double angleDisparity2(const geometry_msgs::Quaternion& quat1,
                       const geometry_msgs::Quaternion& quat2);
tf::Vector3 getDirVector(const geometry_msgs::Quaternion& quat);
geometry_msgs::Quaternion rotateQuat(const geometry_msgs::Quaternion& quat,
                                     const double& angleZ);

double euclideanDistanceLine(const geometry_msgs::Point& point,
                             const geometry_msgs::Point& point1,
                             const geometry_msgs::Point& point2);


geometry_msgs::Point32 transformPoint(const geometry_msgs::Point32& point, double x, double y, double theta);
geometry_msgs::Point transformPoint(const geometry_msgs::Point& point, double x, double y, double theta);

} // namespace geom_utils
#endif