#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import numpy.linalg as lin
import math

class LinearKalman(object) : 
    
    def __init__(self) :
        self._imu_sub = rospy.Subscriber("mavros/imu/data_raw", Imu, self.CallBack)
        self._euler_sub = rospy.Subscriber("accel_to_euler", Vector3, self.EulerCallBack)
        self._kalman_pub = rospy.Publisher("kalman", Vector3, queue_size = 10)
        self._kalman_pose_pub = rospy.Publisher("kalman_pose", PoseStamped, queue_size=10)
        self._dt = 0.02
        self._H = np.eye(4)
        self._Q = 0.0001*np.eye(4)
        self._R = 10*np.eye(4)
        self._P = np.eye(4)
        self._x = np.array([[1],[0],[0],[0]])
        self._A = np.zeros((4,4))
        self._K = np.zeros((4,4))
        self._xp = np.zeros((4,1))
        self._Pp = np.zeros((4,4))
        self._z = np.zeros((4,1))
        self._degree = Vector3()
        self._euler = Vector3()
        self._pose = PoseStamped()

        self._pose.header.stamp = rospy.Time.now()
        self._pose.header.frame_id = "base_link"

    def CallBack(self, msg) :
        self.Create_A(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        self.Prediction()
        self.GetKalmanGain()
        
        self._x = self._xp + self._K.dot(self._z - self._H.dot(self._xp))
        self._P = self._Pp - self._K.dot(self._H.dot(self._Pp))

        self._euler.x = math.atan2( 2*(self._x[2][0]*self._x[3][0] + self._x[0][0]*self._x[1][0]), 1 - 2*(self._x[1][0]**2 + self._x[2][0]**2))
        self._euler.y = -math.asin( 2*(self._x[1][0]*self._x[3][0] - self._x[0][0]*self._x[2][0]))
        self._euler.z = math.atan2( 2*(self._x[1][0]*self._x[2][0] + self._x[0][0]*self._x[3][0]), 1 - 2*(self._x[2][0]**2 + self._x[3][0]**2))

        self._degree.x = self._euler.x * 180/math.pi
        self._degree.y = self._euler.y * 180/math.pi
        self._degree.z = self._euler.z * 180/math.pi
        
        quater = quaternion_from_euler(self._euler.x, self._euler.y, self._euler.z)
        self._pose.pose.position.x = 0
        self._pose.pose.position.y = 0
        self._pose.pose.position.z = 0
        self._pose.pose.orientation.x = quater[0]
        self._pose.pose.orientation.y = quater[1]
        self._pose.pose.orientation.z = quater[2]
        self._pose.pose.orientation.w = quater[3]

        self._kalman_pub.publish(self._degree)
        self._kalman_pose_pub.publish(self._pose)
        #print(self._degree)

    def EulerCallBack(self, msg) :
        phi = msg.x*math.pi/180
        theta = msg.y*math.pi/180
        psi = msg.z*math.pi/180
        self._z = quaternion_from_euler(phi, theta, 0)
    
    def Prediction(self) :
        self._xp = self._A.dot(self._x)
        self._Pp = self._A.dot(self._P.dot(self._A.T)) + self._Q

    def GetKalmanGain(self) :
        temp = self._H.dot(self._Pp.dot(self._H.T)) + self._R
        inverse_temp = lin.inv(temp)
        self._K = self._Pp.dot(self._H.T.dot(inverse_temp))

    def Create_A(self, p, q, r) :
        A = np.zeros((4,4))
        A[0][1] = -p
        A[0][2] = -q
        A[0][3] = -r
        A[1][0] = p
        A[1][2] = r
        A[1][3] = -q
        A[2][0] = q
        A[2][1] = -r
        A[2][3] = p
        A[3][0] = r
        A[3][1] = q
        A[3][2] = -p
        I = np.eye(4)
        self._A = I+self._dt/2*A

if __name__ == '__main__' :
    rospy.init_node("euler_kalman")
    LinearKalman()
    rospy.spin()