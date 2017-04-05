#!/usr/bin/env python

import rospy
import sys
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3

class FILTER:
    def __init__(self):
        self.ekf_sub = rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.listen)
        self.ekf_pub = rospy.Publisher('/ekf_pub', Vector3, queue_size = 1)

    def listen(self, msg):
        quaternions = msg.pose.pose.orientation
        temp = (quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        euler = euler_from_quaternion(temp)
        eulerV = Vector3()
        eulerV.x = euler[0]
        eulerV.y = euler[1]
        eulerV.z = euler[2]
        self.ekf_pub.publish(eulerV)


def main():
    rospy.init_node('ekf_filter_data')
    listen = FILTER()
    rospy.spin()

if __name__ == '__main__':
    main()
