#include <scat_libs/tf_utils.h>

namespace tf_utils
{

geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::StampedTransform &transform)
{
  tf::Vector3 v(point.x, point.y, 0);
  v = transform * v;
  return rosmsg::makePoint(v.x(), v.y(), v.z());
}

geometry_msgs::Point32 transformPoint(const geometry_msgs::Point32 &point, const tf::StampedTransform &transform)
{
  tf::Vector3 v(point.x, point.y, point.z);
  v = transform * v;
  return rosmsg::makePoint32(v.x(), v.y(), v.z());
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf::StampedTransform &transform)
{
  geometry_msgs::Pose new_pose;
  tf::Transform pose_tf, new_tf;
  tf::poseMsgToTF(pose, pose_tf);
  new_tf = transform * pose_tf;
  tf::poseTFToMsg(new_tf, new_pose);
  return new_pose;
}

geometry_msgs::PoseArray transformPoseArray(const geometry_msgs::PoseArray &posearray, 
const std::string &target_frame, const tf::TransformListener& tf_listener)
{
  tf::StampedTransform transform;
  try {
    tf_listener.waitForTransform(target_frame, posearray.header.frame_id, posearray.header.stamp, ros::Duration(0.1));
    tf_listener.lookupTransform(target_frame, posearray.header.frame_id, posearray.header.stamp, transform);
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR_STREAM(ex.what());
    return posearray;
  }  

  geometry_msgs::PoseArray new_posearray;
  new_posearray.header.frame_id = target_frame;
  new_posearray.header.stamp = posearray.header.stamp;
  for (auto& pose : posearray.poses)
  {
    geometry_msgs::Pose new_pose;
    tf::Transform pose_tf, new_tf;
    tf::poseMsgToTF(pose, pose_tf);
    new_tf = transform * pose_tf;
    tf::poseTFToMsg(new_tf, new_pose);    
    new_posearray.poses.push_back(new_pose);
  }

  return new_posearray;
}

} // namespace tf_utils
