#!/usr/bin/env python

import rospy

from math import pi
from std_msgs.msg import Float64
#from aruco_ros.msg import MarkerArray
#from aruco_ros.msg import Marker
from ros_pololu_servo.msg import MotorState as MS
from ros_pololu_servo.msg import MotorStateList as MSL
from ros_pololu_servo.msg import MotorCommand as MC

BASE_SPEED = 14.0
CREEP_SPEED = 0.05
GUNIT_DIST = 1.0 # ???? CHANGE ME
MAX_SPEED = 100.0
GUNIT_DIR = 1.2 # ??? CHANGE ME


class Aruco_Throt:
    def __init__(self):
        self.aruco_sub = rospy.Subscriber('aruco_single/distance', Float64, self.listen)
        self.setpoint_speed_pub = rospy.Publisher('throttle/setpoint', Float64, queue_size = 1)
        self.state_speed_pub = rospy.Publisher('throttle/state', Float64, queue_size = 1)
        self.speed = BASE_SPEED*pi/400

    def listen(self, msg):
        if msg.data > GUNIT_DIST:
            self.speed += CREEP_SPEED*(BASE_SPEED/self.speed)
            rospy.setpoint_speed_pub.publish(self.speed)
            rospy.state_speed_pub(0.0)
        else:
            # point of no return
            rate = rospy.Rate(10) # 10hz
            for i in xrange(int(GUNIT_DIR * 10)):
                rospy.setpoint_speed_pub.publish(MAX_SPEED*pi/400)
                rospy.state_speed_pub(0.0)
                rate.sleep()
            while True:
                rospy.setpoint_speed_pub.publish(0.0)
                rospy.state_speed_pub(0.0)
                rate.sleep()
            

def main():
    rospy.init_node('')
    listen = Aruco_Throt()
    rospy.spin()

if __name__ == '__main__':
    main()
