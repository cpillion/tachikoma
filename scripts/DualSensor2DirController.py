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
    sigs = [80.0,84.0,89.0,99.0,106.0,115.0,129.0,144.0,160.0]
    dists = [142.0,119.0,99.0,74.0,63.0,50.0,40.0,31.0,23.0]
    #print(signal)
    if signal < sigs[0]:
	# assume max distance
        return(dists[0])
    for i in xrange(1,len(sigs)):
	if signal < sigs[i]:
            return((sigs[i]-signal)/(sigs[i]-sigs[i-1])*(dists[i-1]-dists[i])+dists[i])
    # max sensitivity:
    return dists[-1]

def RightIR2Distance(signal):
    # crude linear interpolation, in inches
    sigs = [76.0,82.0,91.0,101.0,106.0,116.0,128.0,141.0,160.0]
    dists = [142.0,119.0,99.0,74.0,63.0,50.0,40.0,31.0,22.0]
    #print(signal)
    if signal < sigs[0]:
	# assume max distance
        return(dists[0])
    for i in xrange(1,len(sigs)):
	if signal < sigs[i]:
            return((sigs[i]-signal)/(sigs[i]-sigs[i-1])*(dists[i-1]-dists[i])+dists[i])
    # max sensitivity:
    return dists[-1]


HISTORY = 4 # smooths responses, but also slows reaction, needs to be > 1
BUFFER = 12 # targeted distance to avoid in inches
STEERING_SENSITIVITY = 0.5 # rescaling factor

MINMOVE_ITERATIONS = 20 #
MINMOVE            = 0.05 # fractional amount
THROTTLE_SENSITIVITY = 0.005 # rescaling factor
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
        self.steering_last = 0
        self.minmove_L = []
        self.minmove_R = []
        self.minmove_old = 0
        self.coast_speed = 0.0
        self.min_speed = 24
        self.max_speed = 30#1.5
        self.throttle_in = 0.0

    def listen(self, msg):
        # message can have multiple motor states
        for state in msg.motor_states:
	    if state.name == 'LeftIR':
	        if len(self.dist_L) < HISTORY:
                    # only update on real signal
                    if state.pulse:
                        self.dist_L.append(float(state.pulse))
                    else:
                        print("No LeftIR signal")
                        self.dist_L.append(self.dist_L[-1])
	        else:
                    # only update on real signal
                    if state.pulse:
                        self.dist_L[self.oldest] = float(state.pulse)
                    else:
                        print("No LeftIR signal")
                        self.dist_L[self.oldest] = self.dist_L[(self.oldest-1)%HISTORY]
            elif state.name == 'RightIR':
	        if len(self.dist_R) < HISTORY:
                    # only update on real signal
                    if state.pulse:
                        self.dist_R.append(float(state.pulse))
                    else:
                        print("No RightIR signal")
                        self.dist_R.append(self.dist_R[-1])
	        else:
                    # only update on real signal
                    if state.pulse:
                        self.dist_R[self.oldest] = float(state.pulse)
                    else:
                        print("No RightIR signal")
                        self.dist_R[self.oldest] = self.dist_R[(self.oldest-1)%HISTORY]
            elif state.name == "Throttle":
                self.throttle_in = float(state.pulse)
	self.oldest = (self.oldest+1) % HISTORY
	# algorithm for value that I think we should care about; may need sensativity tuning
        L = LeftIR2Distance(sum(self.dist_L)/len(self.dist_L))
        R = RightIR2Distance(sum(self.dist_R)/len(self.dist_R))
        value = (L-R)/(min(L,R)-BUFFER)
        # convert to -pi/4 to pi/4 turning radians
        steering = (1/(1+exp(STEERING_SENSITIVITY*value))-0.5)*pi/2
	# publish
        self.steering_setpoint_pub.publish(0.0)
        self.steering_state_pub.publish(steering)
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
            self.minmove_old = (self.minmove_old + 1) % MINMOVE_ITERATIONS
        # decel if close to wall
        if (L < KILL_SOLO or R < KILL_SOLO or
            (L < KILL_DUAL and R < KILL_DUAL)):
            if self.coast_speed > self.min_speed:
                # drop out inverse to speed; build back up later
                self.coast_speed = self.min_speed/self.coast_speed
            elif max(abs(self.minmove_L[self.minmove_old]-L)/L,
                     abs(self.minmove_R[self.minmove_old]-R)/R) < MINMOVE:
                # increase speed very very slowly
                self.coast_speed += 1/(1.0+self.coast_speed)
            else:
                self.coast_speed = 0
            # check for low change situation
        elif self.coast_speed < self.max_speed and (
                max(abs(self.minmove_L[self.minmove_old]-L)/L,
                    abs(self.minmove_R[self.minmove_old]-R)/R) < MINMOVE:
            # increase min speed until a move is first detected
            '''
            if self.min_speed >= self.coast_speed:
            #self.min_speed = self.coast_speed + 1/(1.0+self.coast_speed)
            print("min = %f" % self.min_speed)
            '''
            # increase speed very very slowly
            self.coast_speed += 1/(1.0+self.coast_speed)
            #print("low change coast = %f" % self.coast_speed)
        else:
            # determine how much new turning is being suggested
            #motive = (((abs(steering) - abs(self.steering_last))*2/pi + 0.5))
                      #* (abs(steering)*4/pi))
            # linear interpolation between min and max based on steering
            motive = 1-abs(steering)*4/pi
            #print("motive = %f" % motive)
            self.coast_speed = self.min_speed + motive*(self.max_speed-self.min_speed)
        '''
				(-abs(self.coast_speed-self.min_speed)
                                 + (1-motive)*(1/(1.0+self.coast_speed)
                                               +(self.max_speed-self.coast_speed)*0.1))
        '''
        self.steering_last = steering
        #print(self.coast_speed)
        # track max speed reached, decaying slowly over time
        '''
        if self.coast_speed > self.max_speed:
            self.max_speed = self.coast_speed
	    print(self.max_speed)
        else:
            self.max_speed = self.min_speed + 0.999*(self.max_speed-self.min_speed)
        '''
        # set throttle
        throttle = THROTTLE_SENSITIVITY * self.coast_speed # * (0.5+0.5*(pi/4-abs(steering)))
        self.throttle_setpoint_pub.publish(throttle)
        self.throttle_state_pub.publish(0)
        #print(self.throttle_in)


def main():
    rospy.init_node('setpoint_node')
    listen = SteeringDecider()
    rospy.spin()

if __name__ == '__main__':
    main()
