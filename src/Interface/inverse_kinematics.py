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
from youbot_interface.msg import Floats
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
import forward_kinematics


a = 10   #not too sure about this 
b = 72
c = 150.5
d = 75
e = 155
f = 135
g = 113
h = 105


def inverse_kinematics(xtip, ytip, ztip, rho1=0, rho2, rho3):

	global a, b, c, d, e, f, g, h


	rtip = np.sqrt((np.power(xtip, 2) + np.power(ytip, 2)))
	beta = np.arctan2(ztip, rtip)
	theta = np.arctan2(ytip, xtip)


	Zw2 = b + d 						# Z-coordinate of the joint wrt to world frame
	Z_dash = ztip - Zw2					
	X_dash = rho2						


	Z_dash_dash = Z_dash - (g + h) * np.sin(beta)
	X_dash_dash = X_dash - (g + h) * np.cos(beta)


	#Calculating the value of theta 3
	cos_theta3 = (-np.power(Z_dash_dash, 2) -np.power(X_dash_dash, 2) + np.power(e, 2) + np.power(f, 2)) / (2 * e * f) 
	sin_theta3 = np.sqrt(1 - np.power(cos_theta3, 2))
	q3 = np.arctan2(sin_theta3, cos_theta3)


	#Calculating the value of theta 2
	k1 = e - f * cos_theta3
	k2 = f * sin_theta3
	gamma = arctan2(k1,k2)
	q2 = np.arctan2(Z_dash_dash, X_dash_dash) + gamma

	#Calculating the value of theta4
	q4 = q2 + q3 - beta

	q1 = 0
	q5 = 0

	t = [0, np.deg2rad(q1), np.deg2rad(q2 - 90), np.deg2rad(q3), np.deg2rad(q4), np.deg2rad(90), 0, np.deg2rad(q5)]
	dr = [b, d, 0, 0, 0, 0, 0, h]
	ar = [-c, a, e, f, g, 0, 0, 0]
	alpha = [0, np.deg2rad(-90), 0, 0, 0, 0, np.deg2rad(90), 0]

	tipPos_world = forward_kinematics(q1, q2, q3, q4, q5, 0, 0, theta)					#should have the current position of the robot
	x = xtip - tipPos_world[0, 0]
	y = ytip - tipPos_world[1, 0]


def main():
	rospy.init_node('youbotIK')
	#joints = rospy.Publisher('J-ointStates', Floats, queue_size=10)


	r = rospy.Rate(1000)
	while(running):
		#
		r.sleep()

if __name__=="__main__":
	main()




