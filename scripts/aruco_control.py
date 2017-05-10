#!/usr/bin/env python

import rospy
import sys, signal
import os

from math import pi
from std_msgs.msg import Float64
from ros_pololu_servo.msg import MotorState as MS
from ros_pololu_servo.msg import MotorStateList as MSL
from ros_pololu_servo.msg import MotorCommand as MC

class CTLR_SUB:
    def __init__(self):
        self.left_sub = rospy.Subscriber('control_effort', Float64, self.listen)
        self.motor_pub = rospy.Publisher('pololu/command', MC, queue_size = 1)

    def listen(self, msg):
        '''string joint_name       # Name of the joint (specified in the yaml file), or motor_id for default calibration
        float64 position        # Position to move to in radians
        float32 speed           # Speed to move at (0.0 - 1.0)
        float32 acceleration    # Acceleration to move at (0.0 - 1.0)'''
        output = MC()
        output.joint_name = 'Turning'
        output.position = msg.data
        output.speed = 0.5
        output.acceleration = 0.1
        self.motor_pub.publish(output)

def main():
    rospy.init_node('turning_motor')
    listen = CTLR_SUB()
    rospy.spin()

if __name__ == '__main__':
    main()
