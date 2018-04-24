import numpy as np
from numpy import pi
from numpy import tan,arctan2,sin,cos,sqrt
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
from youbot_interface.msg import Floats
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from forward_kinematics import forward_kinematics


a = 33/1000   #not too sure about this 
b = 140/1000
c = 150.5/1000
d = 147/1000
e = 155/1000
f = 135/1000
g = 112.5/1000
h = 105/1000

def inverse_kinematics(xtip, ytip, ztip, R, rho1, rho2, rho3):

	global a, b, c, d, e, f, g, h

	P5=R[0:3,3]
	P4=P5-
	
	return [q1, q2, q3, q4, q5]


def main():

	R=np.matrix([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00],
 			[  0.00000000e+00,   1.00000000e+00,   0.00000000e+00],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
	
	
	print(inverse_kinematics(-0.1175, 0, 0.5825, R, 0, 0, 0)))

if __name__=="__main__":
	main()




