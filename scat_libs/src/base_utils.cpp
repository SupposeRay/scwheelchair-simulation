#include "scat_libs/rosmsg.h"
#include "scat_libs/base_utils.h"
#include "scat_libs/geom_utils.h"

namespace base_utils 
{

double sign(const double& number)
{
  if (number < 0)
    return -1.0;
  else
    return 1.0;
}

double shiftAngle(const double& angle)
{
    if (angle > M_PI)
        return angle - 2 * M_PI;
    else if (angle < -M_PI)
        return angle + 2 * M_PI;
    else
        return angle;
}

bool checkLimits(const geometry_msgs::Point& point, const double& xmin, const double& xmax, const double& ymin, const double& ymax)
{
  if (point.x > xmin && point.x < xmax && point.y > ymin && point.y < ymax)
    return true;
  else
    return false;
}

bool checkRange(const double& x, const double& y, const double& range)
{
  if (pow(pow(x, 2) + pow(y, 2), 0.5) < range)
    return true;
  else
    return false;
}

bool checkRange(const geometry_msgs::Point& point, const double& range)
{
  if (pow(pow(point.x, 2) + pow(point.y, 2), 0.5) < range)
    return true;
  else
    return false;
}

bool checkRange(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, const double& range)
{
  if (pow(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2), 0.5) < range)
    return true;
}

double euclideanDistance(double point1[2], double point2[2])
{
  return sqrt(pow((point1[0] - point2[0]), 2) + pow((point1[1] - point2[1]), 2));
}

double euclideanDistance(const geometry_msgs::Point& point)
{
  return sqrt(pow((point.x), 2) + pow((point.y), 2));
}

double euclideanDistance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2)
{
  return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
}

double euclideanDistance(const geometry_msgs::Point& point1, const geometry_msgs::Pose2D& point2)
{
  return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
}

double euclideanDistance(const double& x1, const double& y1, const double& x2, const double& y2)
{
  return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

double euclideanDistance(const double& x, const double& y)
{
  return sqrt(pow((x), 2) + pow((y), 2));
}

float euclideanDistance(const geometry_msgs::Point32& point)
{
  return sqrt(pow((point.x), 2) + pow((point.y), 2));
}

float euclideanDistance(const float& x1, const float& y1, const float& x2, const float& y2)
{
  return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

float euclideanDistance(const float& x, const float& y)
{
  return sqrt(pow((x), 2) + pow((y), 2));
}

//! returns angle from point1 to point2.  This function can be used to calculate heading to nodes from some position in global frame. 
double getAngle(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2)
{
  return atan2(point2.y - point1.y, point2.x - point1.x);
}

//! returns angle to point. This simple function can be used to calculate heading to nodes from local frame. 
double getAngle(const geometry_msgs::Point& point)
{
  return atan2(point.y, point.x);
}

//! returns angle to point32. This simple function can be used to calculate heading to nodes from local frame. 
double getAngle(const geometry_msgs::Point32& point)
{
  return atan2(point.y, point.x);
}

//! returns angle of twist message.
double getAngle(const geometry_msgs::Twist& twist)
{
  return atan2(sign(twist.linear.x)*twist.angular.z, twist.linear.x);
}

} // / namespace base_utils 