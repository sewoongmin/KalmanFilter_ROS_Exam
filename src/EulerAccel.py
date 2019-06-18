#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
import math

g = 9.8
prev_x = np.zeros((3,1))
euler = Vector3()
degree = Vector3()

def CallBack(msg) :
    theta = math.asin(msg.linear_acceleration.x / g)
    phi = math.asin( -msg.linear_acceleration.y / (g*math.cos(theta)))

    euler.x = phi
    euler.y = theta
    euler.z = 0
    
rospy.init_node("euler_accel")
sub = rospy.Subscriber("mavros/imu/data_raw", Imu, CallBack)
pub = rospy.Publisher("accel_to_euler", Vector3, queue_size = 10)
rate = rospy.Rate(50)
while not rospy.is_shutdown() :
    degree.x = euler.x * 180/math.pi
    degree.y = euler.y * 180/math.pi
    degree.z = euler.z * 180/math.pi
    pub.publish(degree)
    rate.sleep()
rospy.spin()