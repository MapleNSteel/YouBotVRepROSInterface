import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos
import vrep
from time import sleep
import signal
import time
import sys
import rospy
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import MultiArrayLayout
import roslib; roslib.load_manifest('youbot_interface')
from youbot_interface.msg import Floats
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter

lx = 0.471/2
ly = 0.300/2
r = 0.02375*2
def mobile_inverse_kinematics(robot_vel_x, robot_vel_y, robot_ang_z):
	global r, lx, ly

	mobile_inv_kin = np.matrix([[1, -1, -(lx + ly)], [1, 1, (lx + ly)], [1, 1, -(lx + ly)], [1, -1, (lx + ly)]])
	wheel_ang_vel = 1/r * mobile_inv_kin * np.matrix([[robot_vel_x], [robot_vel_y], [robot_ang_z]])

	return wheel_ang_vel

