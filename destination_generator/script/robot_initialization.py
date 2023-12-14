#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
# import rospkg
# import tf_conversions
import tf
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import SetMap
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseWithCovarianceStamped

# static_map = OccupancyGrid()

def main():
    rospy.init_node('robot_initialization')

    test_world = rospy.get_param("/test_world")
    # rospy.wait_for_service('/static_map')
    # try:
    #     get_map = rospy.ServiceProxy('/static_map', GetMap)
    #     resp = get_map()
    #     static_map = resp.map
    # except rospy.ServiceException as exc:
    #     print ("\"static_map\" service call failed: " + str(exc))
    # test_world = "hospital"
    # test_world = "test"
    print(test_world)
    state_msg = ModelState()
    pose_msg = PoseWithCovarianceStamped()
    state_msg.model_name = 'mbot'
    if test_world == "hospital":
        state_msg.pose.position.x = -8.5
        state_msg.pose.position.y = -20
        state_msg.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.570823)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0
        pose_msg.pose.pose.position.y = 0
        pose_msg.pose.pose.position.z = 0
        pose_msg.pose.pose.orientation.x = 0
        pose_msg.pose.pose.orientation.y = 0
        pose_msg.pose.pose.orientation.z = 0
        pose_msg.pose.pose.orientation.w = 1
    elif test_world == "test":
        state_msg.pose.position.x = -11
        state_msg.pose.position.y = -4.8
        state_msg.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0
        pose_msg.pose.pose.position.y = 0
        pose_msg.pose.pose.position.z = 0
        pose_msg.pose.pose.orientation.x = 0
        pose_msg.pose.pose.orientation.y = 0
        pose_msg.pose.pose.orientation.z = 0
        pose_msg.pose.pose.orientation.w = 1
    elif test_world == "pomdp_test":
        state_msg.pose.position.x = 3
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -3.14159265358)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0
        pose_msg.pose.pose.position.y = 0
        pose_msg.pose.pose.position.z = 0
        pose_msg.pose.pose.orientation.x = 0
        pose_msg.pose.pose.orientation.y = 0
        pose_msg.pose.pose.orientation.z = 0
        pose_msg.pose.pose.orientation.w = 1
    elif test_world == "corridor":
        state_msg.pose.position.x = -6.70
        state_msg.pose.position.y = 13.50
        state_msg.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.570823)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0
        pose_msg.pose.pose.position.y = 0
        pose_msg.pose.pose.position.z = 0
        pose_msg.pose.pose.orientation.x = 0
        pose_msg.pose.pose.orientation.y = 0
        pose_msg.pose.pose.orientation.z = 0
        pose_msg.pose.pose.orientation.w = 1

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException as exc:
        print ("\"set_model_state\" service call failed: " + str(exc))

    # pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

    # rate = rospy.Rate(5)
    # t_start = rospy.Time.now()
    # while not rospy.is_shutdown():
    #     pub.publish(pose_msg)
    #     t_end = rospy.Time.now()
    #     t_span = (t_end - t_start).to_sec()
    #     if t_span >= 1:
    #         break
    #     rate.sleep()
    
    # rospy.wait_for_service('/set_map')
    # try:
    #     set_map = rospy.ServiceProxy('/set_map', SetMap)
    #     resp = set_map(static_map, pose_msg)
    # except rospy.ServiceException as exc:
    #     print ("\"set_map\" service call failed: " + str(exc))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass