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

MINMOVE_ITERATIONS = 20 #
MINMOVE            = 0.05 # fractional amount
THROTTLE_SENSITIVITY = 0.05 # rescaling factor
KILL_SOLO          = 25
KILL_DUAL          = 40

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
        self.minmove_L = []
        self.minmove_R = []
        self.minmove_old = 0
        self.coast_speed = 0.0
        self.min_speed = 0.0
        self.max_speed = 0.0
        self.throttle = 0.0

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
            elif state.name == "Throttle":
                self.throttle = float(state.pulse)
	self.oldest = (self.oldest+1) % HISTORY
	# algorithm for value that I think we should care about; may need sensativity tuning
        L = LeftIR2Distance(sum(self.dist_L)/len(self.dist_L))
        R = RightIR2Distance(sum(self.dist_R)/len(self.dist_R))
        value = (L-R)/(min(L,R)-BUFFER)
        # convert to -pi/4 to pi/4 turning radians
        value = (1/(1+exp(STEERING_SENSITIVITY*value))-0.5)*pi/2
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
        #
        # update minmove (make sure the car is still moving when we want low speed)
        if len(self.minmove_L) < MINMOVE_ITERATIONS:
            self.minmove_L.append(L)
            self.minmove_R.append(R)
        else:
            self.minmove_L[self.minmove_old] = L
            self.minmove_R[self.minmove_old] = R
        self.minmove_old = (self.minmove_old + 1) % len(self.minmove_L)
        # decel if close to wall
        if L < KILL_SOLO or R < KILL_SOLO or (L < KILL_DUAL and R < KILL_DUAL):
            self.coast_speed = self.min_speed
        # check for low change situation
        if max(abs(self.minmove_L[self.minmove_old]-L)/L,
               abs(self.minmove_R[self.minmove_old]-R)/R) < MINMOVE:
            # increase min speed until a move is first detected
            if self.min_speed >= self.coast_speed:
                self.min_speed = 1/(1.0+self.coast_speed)
            # increase speed very very slowly
            self.coast_speed += 1/(1.0+self.coast_speed)
            #print("low change coast = %f" % self.coast_speed)
        else:
            # plenty of momentum, choose speed based on how 
            # one sensor is much 1.6x the other
            if (L-R)/min(L,R) > 0.6:
                #print("%f %f %f" %(L,R,(L-R)/min(L,R)))
                # one wall is over double other wall, set to min speed and decay
                self.coast_speed = 0.99*min(self.coast_speed, self.min_speed)
                #print("double coast = %f" % self.coast_speed)
            else:
                # increase as long as steering is stable
                # decay to min speed otherwise
	        conflict = pow(1-abs(4/pi*value),2)
                self.coast_speed += (-abs(self.coast_speed-self.min_speed)*0.01*(1-conflict)
                                     + conflict*(1/(1.0+self.coast_speed)
                                                 +(self.max_speed-self.min_speed)*0.01))
                #print("weighted coast = %f" % self.coast_speed)
        # track max speed reached, decaying slowly over time
        if self.coast_speed > self.max_speed:
            self.max_speed = self.coast_speed
        else:
            self.max_speed = self.min_speed + 0.999*(self.max_speed-self.min_speed)
        # set throttle
        value = THROTTLE_SENSITIVITY * self.coast_speed # * (0.5+0.5*(pi/4-abs(value)))
        self.throttle_setpoint_pub.publish(value)
        self.throttle_state_pub.publish(0)
        print(self.throttle)


def main():
    rospy.init_node('setpoint_node')
    listen = SteeringDecider()
    rospy.spin()

if __name__ == '__main__':
    main()
