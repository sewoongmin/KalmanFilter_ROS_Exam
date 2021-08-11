#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import numpy.linalg as lin
import math


