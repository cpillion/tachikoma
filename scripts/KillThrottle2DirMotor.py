#!/usr/bin/env python

import rospy
import sys, signal
import os

from math import pi
from std_msgs.msg import Float64
from ros_pololu_servo.msg import MotorState as MS
from ros_pololu_servo.msg import MotorStateList as MSL
from ros_pololu_servo.msg import MotorCommand as MC


def main():
    rospy.init_node('killing_motor')
    motor_pub= rospy.Publisher('/pololu/command/', MC, queue_size = 3)
    output = MC()
    output.joint_name = 'Throttle'
    output.position = 0
    output.speed = 1.0
    output.acceleration = 1
    motor_pub.publish(output)

if __name__ == '__main__':
    main()
