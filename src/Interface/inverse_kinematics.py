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

	rtip = np.sqrt((np.power(xtip, 2) + np.power(ytip, 2)))
	beta = np.arctan2(R[2,2], sqrt(R[2,0]**2 + R[2,1]**2))
	theta = np.arctan2(R[2,1], R[2,0])


	Zw2 = b + d 						# Z-coordinate of the joint wrt to world frame
	Z_dash = ztip - Zw2					
	X_dash = rho2						


	Z_dash_dash = Z_dash - (g + h) * np.sin(beta)
	X_dash_dash = X_dash - (g + h) * np.cos(beta)


	#Calculating the value of theta 3
	cos_theta3 = (-np.power(Z_dash_dash, 2) -np.power(X_dash_dash, 2) + np.power(e, 2) + np.power(f, 2)) / (2 * e * f) 
	print(cos_theta3)
	sin_theta3 = np.sqrt(1 - np.power(cos_theta3, 2))
	q3 = np.arctan2(sin_theta3, cos_theta3)


	#Calculating the value of theta 2
	k1 = e - f * cos_theta3
	k2 = f * sin_theta3
	gamma = arctan2(k1,k2)
	q2 = np.arctan2(Z_dash_dash, X_dash_dash) + gamma

	#Calculating the value of theta4
	q4 = q2 + q3 - beta

	q1 = rho1
	q5 = 0

	t = [0, (q1), (q2 - pi/2), (q3), (q4), (pi/2), 0, (q5)]
	dr = [b, d, 0, 0, 0, 0, 0, h]
	ar = [-c, a, e, f, g, 0, 0, 0]
	alpha = [0, (-pi/2), 0, 0, 0, 0, (pi/2), 0]

	tipPos_world = forward_kinematics(q1, q2, q3, q4, q5, 0, 0, theta)					#should have the current position of the robot
	x = xtip - tipPos_world[0, 0]
	y = ytip - tipPos_world[1, 0]

	q1=arctan2(sin(q1), cos(q1))
	q2=arctan2(sin(q2), cos(q2))
	q3=arctan2(sin(q3), cos(q3))
	q4=arctan2(sin(q4), cos(q4))
	q5=arctan2(sin(q5), cos(q5))

	return [q1, q2, q3, q4, q5]


def main():

	R=np.matrix([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00],
 [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
	
	
	print(inverse_kinematics(-0.1175, 0, 0.5825, R, 0, 0, 0)))

if __name__=="__main__":
	main()




