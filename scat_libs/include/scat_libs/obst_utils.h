#ifndef SCAT_OBST_UTILS
#define SCAT_OBST_UTILS
// ROS
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
// C++
#include <stdlib.h>
#include <math.h>
#include <vector>
// Package
#include <scat_libs/base_utils.h>
#include <scat_libs/geom_utils.h>
#include <scat_libs/tf_utils.h>
#include <scat_libs/rosmsg.h>
#include <scat_msgs/EnvObjectList.h>
#include <scat_msgs/EnvObject.h>

namespace obst_utils
{

  // This method converts a list of objects (of type EnvObjectList), to vectors of Points in the target_frame.
  std::vector<std::vector<geometry_msgs::Point>> convertObjectListToPoints(const scat_msgs::EnvObjectList &object_list, const std::string &target_frame, const tf::TransformListener &tf_listener);

  // This method converts a list of objects (of type EnvObjectList) into the target_frame.
  scat_msgs::EnvObjectList transformObjectList(const scat_msgs::EnvObjectList &object_list, const std::string &target_frame, const tf::TransformListener &tf_listener);

} // namespace obst_utils
#endif
