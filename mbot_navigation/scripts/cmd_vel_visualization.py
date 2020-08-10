#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import  time

cmd_vel = Twist()
rotation = 0.0
t1 = time()
t2 = time()
dt = 0.0

def cmd_cb(data): 
    global t1, t2, dt, cmd_vel
    t2 = time()
    dt = t2-t1
    cmd_vel = data
    t1 = time()


def odom_cb(data):
    global dt, rotation
    Pose = PoseStamped()
    Pose.header = data.header
    Pose.pose = data.pose.pose    
    if abs(cmd_vel.angular.z) > 0.2:
	rotation += cmd_vel.angular.z * dt / 3.1416
    else:
	rotation = 0.0	
#    Pose.pose.orientation.z += rotation 
#    Pose.pose.orientation.w += rotation 
    rospy.loginfo("desired  rotation is: {}".format(rotation))          
    rospy.loginfo("angular cmd is: {}".format(cmd_vel.angular.z))      
    rospy.loginfo("angular Pose is: {}".format(Pose.pose.orientation.z))      
    cmd_pub.publish(Pose)

rospy.init_node('cmd_vis_node')

odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
cmd_vel_sub = rospy.Subscriber('/cmd_vel_bot', Twist, cmd_cb)
cmd_pub = rospy.Publisher('/cmd_vis', PoseStamped, queue_size=1)

if __name__ == '__main__':
    rospy.spin()
