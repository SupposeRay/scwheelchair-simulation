#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node_handle;

  ros::Rate rate(100);

  tf::TransformBroadcaster broadcaster;

  double p_left_laser_x, p_left_laser_y, p_left_laser_z, p_left_laser_ax, p_left_laser_ay, p_left_laser_az;
  double p_right_laser_x, p_right_laser_y, p_right_laser_z, p_right_laser_ax, p_right_laser_ay, p_right_laser_az;
  double p_imu_x, p_imu_y, p_imu_z, p_imu_ax, p_imu_ay, p_imu_az;
  tf::Vector3 left_laser_pos, right_laser_pos, imu_pos;
  tf::Quaternion left_laser_quat, right_laser_quat, imu_quat;

  if (
      node_handle.getParam("/tf_broadcaster/imu_x", p_imu_x) &
      node_handle.getParam("/tf_broadcaster/imu_y", p_imu_y) &
      node_handle.getParam("/tf_broadcaster/imu_z", p_imu_z) &
      node_handle.getParam("/tf_broadcaster/imu_ax", p_imu_ax) &
      node_handle.getParam("/tf_broadcaster/imu_ay", p_imu_ay) &
      node_handle.getParam("/tf_broadcaster/imu_az", p_imu_az) &

      node_handle.getParam("/tf_broadcaster/left_laser_x", p_left_laser_x) &
      node_handle.getParam("/tf_broadcaster/left_laser_y", p_left_laser_y) &
      node_handle.getParam("/tf_broadcaster/left_laser_z", p_left_laser_z) &
      node_handle.getParam("/tf_broadcaster/left_laser_ax", p_left_laser_ax) &
      node_handle.getParam("/tf_broadcaster/left_laser_ay", p_left_laser_ay) &
      node_handle.getParam("/tf_broadcaster/left_laser_az", p_left_laser_az) &      

      node_handle.getParam("/tf_broadcaster/right_laser_x", p_right_laser_x) &
      node_handle.getParam("/tf_broadcaster/right_laser_y", p_right_laser_y) &
      node_handle.getParam("/tf_broadcaster/right_laser_z", p_right_laser_z) &
      node_handle.getParam("/tf_broadcaster/right_laser_ax", p_right_laser_ax) &
      node_handle.getParam("/tf_broadcaster/right_laser_ay", p_right_laser_ay) &
      node_handle.getParam("/tf_broadcaster/right_laser_az", p_right_laser_az))
    ROS_INFO("Parameters  found.");
  else
  {
    ROS_ERROR("Could not load parameters.");
    ros::requestShutdown();
  }

  imu_pos = tf::Vector3(p_imu_x, p_imu_y, p_imu_z);
  left_laser_pos = tf::Vector3(p_left_laser_x, p_left_laser_y, p_left_laser_z);
  right_laser_pos = tf::Vector3(p_right_laser_x, p_right_laser_y, p_right_laser_z);

  left_laser_quat = tf::createQuaternionFromRPY(p_left_laser_ax, p_left_laser_ay, p_left_laser_az);
  right_laser_quat = tf::createQuaternionFromRPY(p_right_laser_ax, p_right_laser_ay, p_right_laser_az);
  imu_quat = tf::createQuaternionFromRPY(p_imu_ax, p_imu_ay, p_imu_az);
  // rot_quat = tf::createQuaternionFromYaw(3.1416);
  while (node_handle.ok())
  {
    ros::Time timestamp = ros::Time::now();

    // broadcaster.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(rot_quat, tf::Vector3(0.0, 0.0, 0.0)),
    //         timestamp, "base_footprint", "base_link"));

    // broadcaster.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(rot_quat, tf::Vector3(0.0, 0.0, 0.0)),
    //         timestamp, "base_link", "imu"));

    // broadcaster.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(left_laser_quat, left_laser_pos),
    //         timestamp, "base_link", "laser_left"));

    // broadcaster.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(right_laser_quat, right_laser_pos),
    //         timestamp, "base_link", "laser_right"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(imu_quat, imu_pos),
            timestamp, "base_footprint", "imu"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(left_laser_quat, left_laser_pos),
            timestamp, "base_footprint", "laser_left"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(right_laser_quat, right_laser_pos),
            timestamp, "base_footprint", "laser_right"));

    rate.sleep();
  }
}
