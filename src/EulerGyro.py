#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
import math

dt = 0.02
gyro = np.zeros((3,1))
prev_x = np.zeros((3,1))
euler = Vector3()
degree = Vector3()

def CallBack(msg) :
    gyro[0][0] = msg.angular_velocity.x
    gyro[1][0] = msg.angular_velocity.y
    gyro[2][0] = msg.angular_velocity.z

    sinPhi = math.sin(prev_x[0][0])
    cosPhi = math.cos(prev_x[0][0])
    cosTheta = math.cos(prev_x[1][0])
    tanTheta = math.tan(prev_x[1][0])

    phi = prev_x[0][0] + dt*( gyro[0][0] + gyro[1][0]*sinPhi*tanTheta + gyro[2][0]*cosPhi*tanTheta)
    theta = prev_x[1][0] + dt*( gyro[1][0]*cosPhi - gyro[2][0]*sinPhi)
    psi = prev_x[2][0] + dt*(gyro[1][0]*sinPhi/cosTheta + gyro[2][0]*cosPhi/cosTheta)

    prev_x[0][0] = phi
    prev_x[1][0] = theta
    prev_x[2][0] = psi

    euler.x = phi
    euler.y = theta
    euler.z = psi
    
rospy.init_node("euler_gyro")
sub = rospy.Subscriber("mavros/imu/data_raw", Imu, CallBack)
pub = rospy.Publisher("gyro_to_euler", Vector3, queue_size = 10)
rate = rospy.Rate(50)
while not rospy.is_shutdown() :
    degree.x = euler.x * 180/math.pi
    degree.y = euler.y * 180/math.pi
    degree.z = euler.z * 180/math.pi
    pub.publish(degree)
    rate.sleep()
rospy.spin()