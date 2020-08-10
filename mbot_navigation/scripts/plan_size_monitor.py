#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

def local_cb(ldata):
    rospy.loginfo("local plan size: {}".format(len(ldata.poses)))  

def global_cb(gdata):
    rospy.loginfo("global plan size: {}".format(len(gdata.poses)))  

rospy.init_node('plan_node')

global_sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, global_cb)
local_sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, local_cb)

if __name__ == '__main__':
    rospy.spin()
