#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
import rospkg
# import tf_conversions
import tf
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('robot_initialization')

    test_world = rospy.get_param("/test_world")
    # test_world = "hospital"
    # test_world = "test"
    state_msg = ModelState()
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
    elif test_world == "test":
        state_msg.pose.position.x = -11
        state_msg.pose.position.y = -4.8
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass