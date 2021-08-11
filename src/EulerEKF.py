#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import numpy.linalg as lin
import math

class ExtendedKalman(object) : 
    
    def __init__(self) :
        self._imu_sub = rospy.Subscriber("mavros/imu/data_raw", Imu, self.CallBack)
        self._euler_sub = rospy.Subscriber("accel_to_euler", Vector3, self.EulerCallBack)
        self._kalman_pub = rospy.Publisher("ekf", Vector3, queue_size = 10)
        self._kalman_pose_pub = rospy.Publisher("kalman_pose_EKF", PoseStamped, queue_size=10)
        self._dt = 0.02
        self._H = np.zeros((2,3))
        self._H[0][0] = 1
        self._H[1][1] = 1
        self._Q = 0.0001*np.eye(3)
        self._Q[2][2] = 0.1
        self._R = 6*np.eye(2)
        self._P = np.eye(3)
        self._x = np.zeros((3,1))
        self._A = np.zeros((3,3))
        self._K = np.zeros((3,2))
        self._xp = np.zeros((3,1))
        self._Pp = np.zeros((3,3))
        self._z = np.zeros((2,1))
        self._xhat = np.zeros((3,1))
        self._degree = Vector3()
        self._pose = PoseStamped()

        self._pose.header.stamp = rospy.Time.now()
        self._pose.header.frame_id = "base_link"

    def CallBack(self, msg) :

        rates = np.zeros((3,1))
        rates[0][0] = msg.angular_velocity.x
        rates[1][0] = msg.angular_velocity.y
        rates[2][0] = msg.angular_velocity.z

        self.JacobA(self._xhat, rates)
        self.Fx(self._xhat, rates)
        self._Pp = self._A.dot(self._P.dot(self._A.T)) + self._Q

        temp = self._H.dot(self._Pp.dot(self._H.T)) + self._R
        inverse_temp = lin.inv(temp)
        self._K = self._Pp.dot(self._H.T.dot(inverse_temp))
        self._x = self._xp + self._K.dot(self._z - self._H.dot(self._xp))
        self._p = self._Pp - self._K.dot(self._H.dot(self._Pp))
  
        self._degree.x = self._x[0][0] * 180/math.pi
        self._degree.y = self._x[1][0] * 180/math.pi
        self._degree.z = self._x[2][0] * 180/math.pi


        quater = quaternion_from_euler(self._x[0][0], self._x[1][0], self._x[2][0])

        self._pose.pose.position.x = 0
        self._pose.pose.position.y = 0
        self._pose.pose.position.z = 0
        self._pose.pose.orientation.x = quater[0]
        self._pose.pose.orientation.y = quater[1]
        self._pose.pose.orientation.z = quater[2]
        self._pose.pose.orientation.w = -quater[3]
    

    def EulerCallBack(self, msg) :
        phi = msg.x*math.pi/180
        theta = msg.y*math.pi/180
        psi = msg.z*math.pi/180
        self._xhat[0][0] = phi
        self._xhat[1][0] = theta
        self._xhat[2][0] = psi

    def JacobA(self, xhat, rates) :
        A = np.zeros((3, 3))
        phi = xhat[0][0]
        theta = xhat[1][0]

        p = rates[0][0]
        q = rates[1][0]
        r = rates[2][0]

        A[0][0] = q*math.cos(phi)*math.tan(theta) - r*math.sin(phi)*math.tan(theta)
        A[0][1] = q*math.sin(phi)/(math.cos(theta)**2) + r*math.cos(phi)/(math.cos(theta)**2)
        A[1][0] = -q*math.sin(phi) - r* math.cos(phi)
        A[2][0] = q*math.cos(phi)/math.cos(theta) - r*math.sin(phi)/math.cos(theta)
        A[2][1] = q*math.sin(phi)/math.cos(theta)*math.tan(theta) + r*math.cos(phi)/math.cos(theta)*math.tan(theta)

        self._A = np.eye(3) + self._dt*A

    def Fx(self, xhat, rates) :
        phi = xhat[0][0]
        theta = xhat[1][0]

        p = rates[0][0]
        q = rates[1][0]
        r = rates[2][0]

        xdot = np.zeros((3, 1))
        xdot[0][0] = p+q*math.sin(phi)*math.tan(theta) + r*math.cos(phi)*math.tan(theta)
        xdot[1][0] = q*math.cos(phi) - r*math.sin(phi)
        xdot[2][0] = q*math.sin(phi)/math.cos(theta) + r* math.cos(phi)/math.cos(theta)

        self._xp = xhat + self._dt*xdot
    
    def Publish(self) :
        self._kalman_pub.publish(self._degree)
        self._kalman_pose_pub.publish(self._pose)

if __name__ == '__main__' :
    rospy.init_node("euler_ekf")
    rate = rospy.Rate(50)
    test = ExtendedKalman()
    while not rospy.is_shutdown() :
        test.Publish()
        rate.sleep()
    rospy.spin()