#ifndef SCAT_BASE_UTILS
#define SCAT_BASE_UTILS
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
#include <tf/transform_datatypes.h>
// C++
#include <stdlib.h>
#include <math.h>
// Package
#include <scat_libs/geom_utils.h>

namespace base_utils
{
double sign(const double& number);
double shiftAngle(const double& angle);

bool checkLimits(const geometry_msgs::Point& point,
                 const double& xmin, const double& xmax, 
                 const double& ymin, const double& ymax);
bool checkRange(const double& x, const double& y, const double& range);
bool checkRange(const geometry_msgs::Point& point, const double& range);
bool checkRange(const geometry_msgs::Point& point1,
                const geometry_msgs::Point& point2, const double& range);

double euclideanDistance(const geometry_msgs::Point& point);
double euclideanDistance(const geometry_msgs::Point& point1,
                         const geometry_msgs::Point& point2);
double euclideanDistance(const geometry_msgs::Point& point1,
                         const geometry_msgs::Pose2D& point2);
double euclideanDistance(double point1[2], double point2[2]);
double euclideanDistance(const double& x1, const double& y1, const double& x2, const double& y2);
double euclideanDistance(const double& x, const double& y);
float euclideanDistance(const geometry_msgs::Point32& point);
float euclideanDistance(const float& x1, const float& y1, const float& x2, const float& y2);
float euclideanDistance(const float& x, const float& y);

double getAngle(const geometry_msgs::Point& point);
double getAngle(const geometry_msgs::Point32& point);
double getAngle(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2);
double getAngle(const geometry_msgs::Twist& twist);

// Templated functions
template <typename T>
std::vector<T> linspace(T a, T b, size_t N)
{
    T h = (b - a) / static_cast<T>(N - 1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

} // namespace base_utils

#endif