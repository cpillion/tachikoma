#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import sys
import csv
import os
import numpy as np
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

class Pub:
    def __init__(self):
        self.imu_pub = rospy.Publisher('/imu_data', Imu, queue_size = 1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 1)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.listen)

    def listen(self, msg):
        temp = msg
        self.odom_pub.publish(self.pub_odom_msg())
        temp.header.frame_id = 'base_footprint'
        temp.header.stamp = rospy.Time.now()
        self.imu_pub.publish(temp)


    #def pub_msg(self, accel, gyro, time, mag):
    #    msg = Imu()
    #    quat = quaternion_from_euler(gyro[0], gyro[1], gyro[2])
    #    msg.orientation.x = quat[0]
    #    msg.orientation.y = quat[1]
    #    msg.orientation.z = quat[2]
    #    msg.orientation.w = quat[3]
    #    msg.orientation_covariance=[0.0009, 0.0009, 0.0009, 0.0025, 0.0025, 0.0025, 0.003, 0.003, 0.003]
    #    msg.angular_velocity.x = gyro[0]
    #    msg.angular_velocity.y = gyro[1]
    #    msg.angular_velocity.z = gyro[2]
    #    msg.angular_velocity_covariance = [.01]*9
    #    msg.linear_acceleration.x = accel[0]
    #    msg.linear_acceleration.y = accel[1]
    #    msg.linear_acceleration.z = accel[2]
    #    msg.linear_acceleration_covariance = [.01]*9
    #    msg.header.stamp = rospy.Time.now()
    #    self.imu_pub.publish(msg)

    def pub_odom_msg(self):
        msg2 = Odometry()
        poseCov = [0.1, 0, 0, 0, 0, 0,
                    0, 0.1, 0, 0, 0, 0,
                    0, 0, 0.1, 0, 0, 0,
                    0, 0, 0, 99999, 0, 0,
                    0, 0, 0, 0, 99999, 0,
                    0, 0, 0, 0, 0, 99999]
        msg2.header.frame_id = 'base_footprint'
        msg2.pose.pose.position.x = 0
        msg2.pose.pose.position.y = 0
        msg2.pose.pose.position.z = 0
        msg2.pose.pose.orientation.x = 1
        msg2.pose.pose.orientation.y = 0
        msg2.pose.pose.orientation.z = 0
        msg2.pose.pose.orientation.w = 0
        msg2.pose.covariance = poseCov
        msg2.header.stamp = rospy.Time.now()
        return msg2

def main():
    rospy.init_node('imu_node')
    listen = Pub()
    rospy.spin()


if __name__=='__main__':
	main()
