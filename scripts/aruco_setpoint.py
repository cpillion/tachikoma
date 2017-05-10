#!/usr/bin/env python

import rospy
import sys, signal
import os

from math import pi
from std_msgs.msg import Float64
#from aruco_ros.msg import MarkerArray
#from aruco_ros.msg import Marker
from ros_pololu_servo.msg import MotorState as MS
from ros_pololu_servo.msg import MotorStateList as MSL
from ros_pololu_servo.msg import MotorCommand as MC
#from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion as efq
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped

class Aruco_SUB:
    def __init__(self):
        self.aruco_sub = rospy.Subscriber('aruco_single/pose', PoseStamped, self.listen)
        #self.left_sub() = rospy.Subscriber('LeftIR', int8, self.listen)
        #self.setpoint_pos_pub = rospy.Publisher('setpoint_vel', Float64, queue_size = 1)
        #self.state_pos_pub = rospy.Publisher('state_vel', Float64, queue_size = 1)
        self.setpoint_angle_pub = rospy.Publisher('setpoint', Float64, queue_size = 1)
        self.state_angle_pub = rospy.Publisher('state', Float64, queue_size = 1)
        #self.setpoint_pub(128/512-pi/4)
        self.dist_pub = rospy.Publisher("aruco_single/distance", Float64, queue_size = 1)

    def listen(self, msg):
#        quaternion = [0, 0, 0, 0]
#        quaternion[0] = msg.pose.orientation.x
#        quaternion[1] = msg.pose.orientation.y
#        quaternion[2] = msg.pose.orientation.z
#        quaternion[3] = msg.pose.orientation.w
        euler = efq((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, 	msg.pose.orientation.w))
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        #self.setpoint_pos_pub.publish(0)
        #self.state_pos_pub.publish(z)
        self.setpoint_angle_pub.publish(0)
        self.state_angle_pub.publish(x - (0.531*z+0.02043))
        self.dist_pub.publish(z)

def main():
    rospy.init_node('setpoint_node')
    listen = Aruco_SUB()
    rospy.spin()

if __name__ == '__main__':
    main()
