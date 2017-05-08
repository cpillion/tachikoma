#!/usr/bin/env python

import rospy
import sys, signal
import os

from math import pi
from std_msgs.msg import Float64
from ros_pololu_servo.msg import MotorState as MS
from ros_pololu_servo.msg import MotorStateList as MSL
from ros_pololu_servo.msg import MotorCommand as MC

PUBTHRESH = 0.01 # don't bother publishing if the last publish is within this percent

class CTLR_SUB:
    def __init__(self):
        self.steer_sub = rospy.Subscriber('steering/control_effort', Float64, self.steer_listen)
        self.throttle_sub = rospy.Subscriber('throttle/control_effort', Float64,
                                             self.throttle_listen)
        self.motor_pub= rospy.Publisher('/pololu/command/', MC, queue_size = 3)
        self.steer_last = 0.0
        self.throttle_last = 0.0

    def steer_listen(self, msg):
        '''string joint_name       # Name of the joint (specified in the yaml file), or motor_id for default calibration
        float64 position        # Position to move to in radians
        float32 speed           # Speed to move at (0.0 - 1.0)
        float32 acceleration    # Acceleration to move at (0.0 - 1.0)'''
        if abs(self.steer_last-msg.data)/(abs(msg.data)+0.001) > PUBTHRESH:
            output = MC()
            output.joint_name = 'Turning'
            output.position = msg.data
            output.speed = 1.0
            output.acceleration = 0.3
            self.motor_pub.publish(output)
            self.steer_last = msg.data

    def throttle_listen(self, msg):
        '''string joint_name       # Name of the joint (specified in the yaml file), or motor_id for default calibration
        float64 position        # Position to move to in radians
        float32 speed           # Speed to move at (0.0 - 1.0)
        float32 acceleration    # Acceleration to move at (0.0 - 1.0)'''
        if abs(self.throttle_last-msg.data)/(abs(msg.data)+0.001) > PUBTHRESH:
            output = MC()
            output.joint_name = 'Throttle'
            output.position = msg.data
            output.speed = 1.0
            output.acceleration = 0.5
            self.motor_pub.publish(output)
            self.throttle_last = msg.data

def main():
    rospy.init_node('turning_motor')
    listen = CTLR_SUB()
    rospy.spin()

if __name__ == '__main__':
    main()
