#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_tf_broadcaster");
  ros::NodeHandle node_handle;

  ros::Rate rate(10);

  tf::TransformBroadcaster broadcaster;

  while (node_handle.ok())
  {
    ros::Time timestamp = ros::Time::now();

    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
             tf::Vector3(0, 0, 0)),
            timestamp, "base_footprint", "base_scan"));

    rate.sleep();
  }
}
