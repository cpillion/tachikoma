#!/usr/bin/env python

import rospy
import sys, signal
import os

from math import pi, exp
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from ros_pololu_servo.msg import MotorState as MS
from ros_pololu_servo.msg import MotorStateList as MSL
from ros_pololu_servo.msg import MotorCommand as MC



def LeftIR2Distance(signal):
    # crude linear interpolation, in inches
    #print(signal)
    if signal < 78:
	# assume max distance
	return(144.0)
    elif signal < 83:
	return((83.0-signal)/(83-78)*(144-112)+112)
    elif signal < 96:
	return((96.0-signal)/(96-83)*(112-74)+74)
    elif signal < 103:
	return((103.0-signal)/(103-96)*(74-61)+61)
    elif signal < 122:
	return((122.0-signal)/(122-103)*(61-39)+39)
    elif signal < 143:
	return((143.0-signal)/(143-122)*(39-29)+29)
    elif signal < 157:
	return((157.0-signal)/(157-143)*(29-22)+22)
    # max sensitivity
    return(22.0)

def RightIR2Distance(signal):
    # crude linear interpolation, inches
    if signal < 75:
	# assume max distance
	return(143.0)
    elif signal < 81:
	return((81.0-signal)/(81-75)*(143-113)+113)
    elif signal < 87:
	return((87.0-signal)/(87-81)*(113-88)+88)
    elif signal < 96:
	return((96.0-signal)/(96-87)*(88-70)+70)
    elif signal < 108:
	return((108.0-signal)/(108-96)*(70-55)+55)
    elif signal < 129:
	return((129.0-signal)/(129-108)*(55-37)+37)
    elif signal < 154:
	return((154.0-signal)/(154-129)*(37-24)+24)
    # max snesitivity
    return(22.0)


HISTORY = 4 # smooths responses, but also slows reaction, needs to be > 1
BUFFER = 12 # targeted distance to avoid in inches
STEERING_SENSITIVITY = 0.5 # rescaling factor

class SteeringDecider:
    def __init__(self):
        self.states_sub = rospy.Subscriber('/pololu/motor_states', MSL, self.listen)
	self.steering_setpoint_pub = rospy.Publisher("steering/setpoint", Float64, queue_size=1)
        self.steering_state_pub = rospy.Publisher("steering/state", Float64, queue_size=1)
	self.throttle_setpoint_pub = rospy.Publisher("throttle/setpoint", Float64, queue_size=1)
        self.throttle_state_pub = rospy.Publisher("throttle/state", Float64, queue_size=1)
        self.dist_L = []
        self.dist_R = []
        self.oldest = 0

    def listen(self, msg):
        # message can have multiple motor states
        for state in msg.motor_states:
	    if state.name == 'LeftIR':
	        if len(self.dist_L) < HISTORY:
	            self.dist_L.append(float(state.pulse))
	        else:
	            self.dist_L[self.oldest] = float(state.pulse)
            elif state.name == 'RightIR':
	        if len(self.dist_R) < HISTORY:
	            self.dist_R.append(float(state.pulse))
	        else:
	            self.dist_R[self.oldest] = float(state.pulse)
	self.oldest = (self.oldest+1) % HISTORY
	# algorithm for value that I think we should care about; may need sensativity tuning
        L = LeftIR2Distance(sum(self.dist_L)/len(self.dist_L))
        R = RightIR2Distance(sum(self.dist_R)/len(self.dist_R))
        value = (L-R)/(min(L,R)-BUFFER)
        # convert to -pi/4 to pi/4 turning radians
        value = (1/(1+exp(STEERING_SENSITIVITY*value))-0.5)*pi/4
	# publish
        self.steering_setpoint_pub.publish(0.0)
        self.steering_state_pub.publish(value)
        ##########
        # throttle algorithm
        # goal is to go as fast as possible while still being able to make turns
        # max speed dictated by corner sharpness and how early they can be detected
        # corner sharpness dictated by hallway width; use current width to estimate next?
        # * drop speed when an IR sensor drops out or is very close
        # * increase when sensors are medium and close
        # * base speed should be enough that sensors change when not equal
        # * max speed should be ???? maybe based on last base speed? (except nonlinear)
        #if (L-R)/min(L,R) > 1:
            # one sensor is more than double the other
            # slow until just fast enough to still see change (nonlinear is tricky)
            # last
            #L1 = sum(self.dist_L[(i+self.oldest) % len(self.dist_L) for ]
            #if 
	value = 0.1*pi/4-abs(value)
        self.throttle_setpoint_pub.publish(value)
	self.throttle_state_pub.publish(0)


def main():
    rospy.init_node('setpoint_node')
    listen = SteeringDecider()
    rospy.spin()

if __name__ == '__main__':
    main()
