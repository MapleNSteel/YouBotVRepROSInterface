import numpy as np

a = 10/1000.   #not too sure about this 
b = 72/1000.
c = 150.5/1000.
d = 75/1000.
e = 155/1000.
f = 135/1000.
g = 113/1000.
h = 105/1000.


def manipulator_kinematics(q1, q2, q3, q4, q5, q1d, q2d, q3d, q4d, q5d, q6d, fx, fy, fz, mx, my, mz, theta):
	global a, b, c, d, e, f, g, h

	J = np.matrix([[- 1.0*a*sin(q1) - 1.0*h*(6.12e-17*cos(q1) + cos(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1)) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1))) - 1.0*g*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1)) - 1.0*e*cos(q2 - 1.57)*sin(q1) - 6.12e-17*e*sin(q2 - 1.57)*cos(q1) - 1.0*f*sin(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1)) - 1.0*g*cos(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1)) - 1.0*f*cos(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)), - 1.0*h*(1.0*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) + sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1))) - 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) - 1.0*g*sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) - 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*sin(q1) + 1.0*sin(q2 - 1.57)*cos(q1)) - 6.12e-17*e*cos(q2 - 1.57)*sin(q1) - 1.0*e*sin(q2 - 1.57)*cos(q1) - 1.0*f*sin(q3)*(1.0*cos(q2 - 1.57)*cos(q1) - 6.12e-17*sin(q2 - 1.57)*sin(q1)), - 1.0*h*(1.0*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) + sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1))) - 1.0*f*sin(q3)*(cos(q2 - 1.57)*cos(q1) - 6.12e-17*sin(q2 - 1.57)*sin(q1)) - 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) - 1.0*g*sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) - 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*sin(q1) + sin(q2 - 1.57)*cos(q1)), -4.0e-33*(g + h)*(1.25e32*sin(q2 - 1.0*q1 + q3 + q4 - 1.57) + 1.25e32*sin(q1 + q2 + q3 + q4 - 1.57)), 0], [a*cos(q1) - 1.0*h*(6.12e-17*sin(q1) - 1.0*cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1))) - 1.0*f*sin(q3)*(6.12e-17*cos(q2 - 1.57)*sin(q1) + sin(q2 - 1.57)*cos(q1)) + e*cos(q2 - 1.57)*cos(q1) - 6.12e-17*e*sin(q2 - 1.57)*sin(q1) + 1.0*g*cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) - 1.0*g*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1)) + 1.0*f*cos(q3)*(cos(q2 - 1.57)*cos(q1) - 6.12e-17*sin(q2 - 1.57)*sin(q1)), 0.5*h*cos(q1 + q2 + q3 + q4 - 1.57) - 0.5*h*cos(q2 - 1.0*q1 + q3 + q4 - 1.57) - 1.0*f*sin(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) + 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1)) + 6.12e-17*e*cos(q2 - 1.57)*cos(q1) + 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1)) - 1.0*e*sin(q2 - 1.57)*sin(q1) - 1.0*g*sin(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1)), 0.5*h*cos(q1 + q2 + q3 + q4 - 1.57) - 0.5*h*cos(q2 - 1.0*q1 + q3 + q4 - 1.57) - 1.0*f*sin(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) + 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1)) + 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1)) - 1.0*g*sin(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1)), -1.0e-49*(g + h)*(1.0e49*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 6.12e32*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) + 6.12e32*cos(q2 - 1.57)*cos(q1)*sin(q3)*sin(q4) + 1.0e49*cos(q2 - 1.57)*cos(q3)*sin(q1)*sin(q4) + 6.12e32*sin(q2 - 1.57)*cos(q1)*cos(q3)*sin(q4) - 1.0e49*sin(q2 - 1.57)*sin(q1)*sin(q3)*sin(q4)), 0], [ 0,  - 1.0*g*cos(q2 + q3 + q4 - 1.57) - 1.0*h*cos(q2 + q3 + q4 - 1.57) - 2.56e-12*e*cos(q2) - 1.0*e*sin(q2) - 1.0*f*cos(q2 + q3 - 1.57),  - 1.0*g*cos(q2 + q3 + q4 - 1.57) - 1.0*h*cos(q2 + q3 + q4 - 1.57) - 1.0*f*cos(q2 + q3 - 1.57),  -1.0*cos(q2 + q3 + q4 - 1.57)*(g + h), 0], [ 0, -1.0*sin(q1),-1.0*sin(q1), -1.0*sin(q1), cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) - 6.12e-17*sin(q1) - 1.0*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1))], [ 0, cos(q1), cos(q1), cos(q1), cos(q4)*(cos(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) + sin(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1))) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1))], [1.0, 0, 0,  0, 1.0*sin(q2 + q3 + q4 - 1.57)]])


	vel_kin_arm = np.matrix([[- 1.0*q2d*(1.0*h*(1.0*cos(q4)*(1.0*sin(q2 + q3 - 1.57)*cos(q1)) + sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1))) + 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) + 1.0*g*sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) + 1.0*f*cos(q3)*(1.0*sin(q2 - 1.57)*cos(q1)) + 6.12e-17*e*cos(q2 - 1.57)*sin(q1) + 1.0*e*sin(q2 - 1.57)*cos(q1) + 1.0*f*sin(q3)*(1.0*cos(q2 - 1.57)*cos(q1) - 6.12e-17*sin(q2 - 1.57)*sin(q1))) - 1.0*q1d*(1.0*a*sin(q1) + 1.0*h*(6.12e-17*cos(q1) + cos(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1)) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1))) + 1.0*g*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1)) + 1.0*e*cos(q2 - 1.57)*sin(q1) + 6.12e-17*e*sin(q2 - 1.57)*cos(q1) + 1.0*f*sin(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1)) + 1.0*g*cos(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1)) + 1.0*f*cos(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1))) - 1.0*q3d*(1.0*h*(1.0*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) + sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1))) + 1.0*f*sin(q3)*(cos(q2 - 1.57)*cos(q1) - 6.12e-17*sin(q2 - 1.57)*sin(q1)) + 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + 1.0*sin(q2 + q3 - 1.57)*cos(q1)) + 1.0*g*sin(q4)*(1.0*cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) + 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*sin(q1) + sin(q2 - 1.57)*cos(q1))) - 4.0e-33*q4d*(g + h)*(1.25e32*sin(q2 - 1.0*q1 + q3 + q4 - 1.57) + 1.25e32*sin(q1 + q2 + q3 + q4 - 1.57))], [q1d*(a*cos(q1) - 1.0*h*(6.12e-17*sin(q1) - 1.0*cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1))) - 1.0*f*sin(q3)*(6.12e-17*cos(q2 - 1.57)*sin(q1) + sin(q2 - 1.57)*cos(q1)) + e*cos(q2 - 1.57)*cos(q1) - 6.12e-17*e*sin(q2 - 1.57)*sin(q1) + 1.0*g*cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) - 1.0*g*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1)) + 1.0*f*cos(q3)*(cos(q2 - 1.57)*cos(q1) - 6.12e-17*sin(q2 - 1.57)*sin(q1))) - 1.0*q2d*(0.5*h*cos(q2 - 1.0*q1 + q3 + q4 - 1.57) - 0.5*h*cos(q1 + q2 + q3 + q4 - 1.57) + 1.0*f*sin(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) - 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1)) - 6.12e-17*e*cos(q2 - 1.57)*cos(q1) - 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1)) + 1.0*e*sin(q2 - 1.57)*sin(q1) + 1.0*g*sin(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1))) - 1.0*q3d*(0.5*h*cos(q2 - 1.0*q1 + q3 + q4 - 1.57) - 0.5*h*cos(q1 + q2 + q3 + q4 - 1.57) + 1.0*f*sin(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) - 1.0*g*cos(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1)) - 1.0*f*cos(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1)) + 1.0*g*sin(q4)*(cos(q2 + q3 - 1.57)*sin(q1) + 6.12e-17*sin(q2 + q3 - 1.57)*cos(q1))) - 1.0e-49*q4d*(g + h)*(1.0e49*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 6.12e32*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) + 6.12e32*cos(q2 - 1.57)*cos(q1)*sin(q3)*sin(q4) + 1.0e49*cos(q2 - 1.57)*cos(q3)*sin(q1)*sin(q4) + 6.12e32*sin(q2 - 1.57)*cos(q1)*cos(q3)*sin(q4) - 1.0e49*sin(q2 - 1.57)*sin(q1)*sin(q3)*sin(q4))], [- 1.0*q2d*(1.0*g*cos(q2 + q3 + q4 - 1.57) + 1.0*h*cos(q2 + q3 + q4 - 1.57) + 2.56e-12*e*cos(q2) + 1.0*e*sin(q2) + 1.0*f*cos(q2 + q3 - 1.57)) - 1.0*q3d*(1.0*g*cos(q2 + q3 + q4 - 1.57) + 1.0*h*cos(q2 + q3 + q4 - 1.57) + 1.0*f*cos(q2 + q3 - 1.57)) - 1.0*q4d*cos(q2 + q3 + q4 - 1.57)*(g + h)], [- 1.0*q5d*(6.12e-17*sin(q1) - 1.0*cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) + 1.0*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1))) - 1.0*q2d*sin(q1) - 1.0*q3d*sin(q1) - 1.0*q4d*sin(q1)], [q5d*(6.12e-17*cos(q1) + cos(q4)*(cos(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) + sin(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1))) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1))) + q2d*cos(q1) + q3d*cos(q1) + q4d*cos(q1)], [1.0*q1d + 6.12e-17*q2d + 6.12e-17*q3d + 6.12e-17*q4d - 1.0*q5d*(1.0*sin(q2 + q3 + q4 - 1.57) - 3.75e-33)]])


	Torque_arm = np.matrix([[mz - 6.12e-17*fx*h*cos(q1) - 1.0*a*fx*sin(q1) - 6.12e-17*fy*h*sin(q1) + a*fy*cos(q1) + e*fy*cos(q2 - 1.57)*cos(q1) - 1.0*e*fx*cos(q2 - 1.57)*sin(q1) - 6.12e-17*e*fx*sin(q2 - 1.57)*cos(q1) - 6.12e-17*e*fy*sin(q2 - 1.57)*sin(q1) - 6.12e-17*fy*h*cos(q2 + q3 - 1.57)*sin(q1)*sin(q4) - 1.0*fy*h*sin(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 6.12e-17*fy*h*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1) + fx*g*sin(q2 + q3 - 1.57)*sin(q1)*sin(q4) + fx*h*sin(q2 + q3 - 1.57)*sin(q1)*sin(q4) + f*fy*cos(q2 - 1.57)*cos(q1)*cos(q3) - 6.12e-17*f*fx*cos(q2 - 1.57)*cos(q1)*sin(q3) - 1.0*f*fx*cos(q2 - 1.57)*cos(q3)*sin(q1) - 6.12e-17*f*fx*sin(q2 - 1.57)*cos(q1)*cos(q3) - 6.12e-17*f*fy*cos(q2 - 1.57)*sin(q1)*sin(q3) - 1.0*f*fy*sin(q2 - 1.57)*cos(q1)*sin(q3) - 6.12e-17*f*fy*sin(q2 - 1.57)*cos(q3)*sin(q1) + f*fx*sin(q2 - 1.57)*sin(q1)*sin(q3) + fy*g*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) + fy*h*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 6.12e-17*fx*g*cos(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 1.0*fx*g*cos(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 6.12e-17*fx*g*sin(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 6.12e-17*fx*h*cos(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 1.0*fx*h*cos(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 6.12e-17*fx*h*sin(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 6.12e-17*fy*g*cos(q2 + q3 - 1.57)*sin(q1)*sin(q4) - 1.0*fy*g*sin(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 6.12e-17*fy*g*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1)], [6.12e-17*mz + my*cos(q1) - 1.0*mx*sin(q1) - 1.0*e*fz*sin(q2) + 0.5*fy*h*cos(q1 + q2 + q3 + q4 - 1.57) - 1.0*f*fz*cos(q2 + q3 - 1.57) - 1.0*fz*g*cos(q2 + q3 + q4 - 1.57) - 1.0*fz*h*cos(q2 + q3 + q4 - 1.57) - 0.5*fy*h*cos(q2 - 1.0*q1 + q3 + q4 - 1.57) - 2.56e-12*e*fz*cos(q2) + 6.12e-17*e*fy*cos(q2 - 1.57)*cos(q1) - 6.12e-17*e*fx*cos(q2 - 1.57)*sin(q1) - 1.0*e*fx*sin(q2 - 1.57)*cos(q1) - 1.0*e*fy*sin(q2 - 1.57)*sin(q1) + 6.12e-17*fx*g*sin(q2 + q3 - 1.57)*sin(q1)*sin(q4) + 6.12e-17*fx*h*sin(q2 + q3 - 1.57)*sin(q1)*sin(q4) + 6.12e-17*f*fy*cos(q2 - 1.57)*cos(q1)*cos(q3) - 1.0*f*fx*cos(q2 - 1.57)*cos(q1)*sin(q3) - 6.12e-17*f*fx*cos(q2 - 1.57)*cos(q3)*sin(q1) - 1.0*f*fx*sin(q2 - 1.57)*cos(q1)*cos(q3) - 1.0*f*fy*cos(q2 - 1.57)*sin(q1)*sin(q3) - 6.12e-17*f*fy*sin(q2 - 1.57)*cos(q1)*sin(q3) - 1.0*f*fy*sin(q2 - 1.57)*cos(q3)*sin(q1) + 6.12e-17*f*fx*sin(q2 - 1.57)*sin(q1)*sin(q3) + 6.12e-17*fy*g*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 1.0*fx*g*cos(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 6.12e-17*fx*g*cos(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 1.0*fx*g*sin(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 1.0*fx*h*cos(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 6.12e-17*fx*h*cos(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 1.0*fx*h*sin(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 1.0*fy*g*cos(q2 + q3 - 1.57)*sin(q1)*sin(q4) - 6.12e-17*fy*g*sin(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 1.0*fy*g*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1)], [6.12e-17*mz + my*cos(q1) - 1.0*mx*sin(q1) + 0.5*fy*h*cos(q1 + q2 + q3 + q4 - 1.57) - 1.0*f*fz*cos(q2 + q3 - 1.57) - 1.0*fz*g*cos(q2 + q3 + q4 - 1.57) - 1.0*fz*h*cos(q2 + q3 + q4 - 1.57) - 0.5*fy*h*cos(q2 - 1.0*q1 + q3 + q4 - 1.57) + 6.12e-17*fx*g*sin(q2 + q3 - 1.57)*sin(q1)*sin(q4) + 6.12e-17*fx*h*sin(q2 + q3 - 1.57)*sin(q1)*sin(q4) + 6.12e-17*f*fy*cos(q2 - 1.57)*cos(q1)*cos(q3) - 1.0*f*fx*cos(q2 - 1.57)*cos(q1)*sin(q3) - 6.12e-17*f*fx*cos(q2 - 1.57)*cos(q3)*sin(q1) - 1.0*f*fx*sin(q2 - 1.57)*cos(q1)*cos(q3) - 1.0*f*fy*cos(q2 - 1.57)*sin(q1)*sin(q3) - 6.12e-17*f*fy*sin(q2 - 1.57)*cos(q1)*sin(q3) - 1.0*f*fy*sin(q2 - 1.57)*cos(q3)*sin(q1) + 6.12e-17*f*fx*sin(q2 - 1.57)*sin(q1)*sin(q3) + 6.12e-17*fy*g*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 1.0*fx*g*cos(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 6.12e-17*fx*g*cos(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 1.0*fx*g*sin(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 1.0*fx*h*cos(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 6.12e-17*fx*h*cos(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 1.0*fx*h*sin(q2 + q3 - 1.57)*cos(q1)*cos(q4) - 1.0*fy*g*cos(q2 + q3 - 1.57)*sin(q1)*sin(q4) - 6.12e-17*fy*g*sin(q2 + q3 - 1.57)*cos(q1)*sin(q4) - 1.0*fy*g*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1)], [6.12e-17*mz + my*cos(q1) - 1.0*mx*sin(q1) - 4.0e-33*fx*(g + h)*(1.25e32*sin(q2 - 1.0*q1 + q3 + q4 - 1.57) + 1.25e32*sin(q1 + q2 + q3 + q4 - 1.57)) - 1.0*fz*cos(q2 + q3 + q4 - 1.57)*(g + h) - 1.0e-49*fy*(g + h)*(1.0e49*sin(q2 + q3 - 1.57)*cos(q4)*sin(q1) - 6.12e32*cos(q2 + q3 - 1.57)*cos(q1)*cos(q4) + 6.12e32*cos(q2 - 1.57)*cos(q1)*sin(q3)*sin(q4) + 1.0e49*cos(q2 - 1.57)*cos(q3)*sin(q1)*sin(q4) + 6.12e32*sin(q2 - 1.57)*cos(q1)*cos(q3)*sin(q4) - 1.0e49*sin(q2 - 1.57)*sin(q1)*sin(q3)*sin(q4))], [my*(6.12e-17*cos(q1) + cos(q4)*(cos(q3)*(cos(q2 - 1.57)*sin(q1) + 6.12e-17*sin(q2 - 1.57)*cos(q1)) + sin(q3)*(6.12e-17*cos(q2 - 1.57)*cos(q1) - 1.0*sin(q2 - 1.57)*sin(q1))) + sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*cos(q1) - 1.0*sin(q2 + q3 - 1.57)*sin(q1))) - 1.0*mx*(6.12e-17*sin(q1) - 1.0*cos(q4)*(cos(q2 + q3 - 1.57)*cos(q1) - 6.12e-17*sin(q2 + q3 - 1.57)*sin(q1)) + 1.0*sin(q4)*(6.12e-17*cos(q2 + q3 - 1.57)*sin(q1) + sin(q2 + q3 - 1.57)*cos(q1))) - 1.0*mz*(1.0*sin(q2 + q3 + q4 - 1.57) - 3.75e-33)]])

	return J, vel_kin_arm, Torque_arm


def combined_kinematics(q1, q2, q3, q4, q5, q1d, q2d, q3d, q4d, q5d, q6d, fx, fy, fz, mx, my, mz, theta):
	combined_J = np.matrix([[0.5*h*sin(q2 - 1.0*q1 + q3 + q4 + 1.0*theta - 1.57) - 1.0*a*sin(q1 - 1.0*theta) - 6.12e-17*h*cos(q1 - 1.0*theta) - 0.5*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 1.0e-49*f*cos(q3)*(6.12e32*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 1.0e49*sin(q1 - 1.0*theta)*cos(q2 - 1.57)) - 1.0e-49*f*sin(q3)*(6.12e32*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 1.0e49*sin(q1 - 1.0*theta)*sin(q2 - 1.57)) - 1.0*g*sin(q4)*(0.5*cos(q1 + q2 + q3 - 1.0*theta - 1.57) - 0.5*cos(q2 - 1.0*q1 + q3 + theta - 1.57)) - 6.12e-17*e*cos(q1 - 1.0*theta)*sin(q2 - 1.57) - 1.0*e*sin(q1 - 1.0*theta)*cos(q2 - 1.57) - 1.0*g*cos(q4)*(0.5*sin(q1 + q2 + q3 - 1.0*theta - 1.57) - 0.5*sin(q2 - 1.0*q1 + q3 + theta - 1.57)), - 0.5*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 1.0e-49*f*cos(q3)*(1.0e49*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 6.12e32*sin(q1 - 1.0*theta)*cos(q2 - 1.57)) - 1.0e-49*f*sin(q3)*(1.0e49*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 6.12e32*sin(q1 - 1.0*theta)*sin(q2 - 1.57)) - 1.0*g*sin(q4)*(0.5*cos(q1 + q2 + q3 - 1.0*theta - 1.57) + 0.5*cos(q2 - 1.0*q1 + q3 + theta - 1.57)) - 1.0*e*cos(q1 - 1.0*theta)*sin(q2 - 1.57) - 6.12e-17*e*sin(q1 - 1.0*theta)*cos(q2 - 1.57) - 1.0*g*cos(q4)*(0.5*sin(q1 + q2 + q3 - 1.0*theta - 1.57) + 0.5*sin(q2 - 1.0*q1 + q3 + theta - 1.57)), 0.5*f*cos(q1 + q2 + q3 - 1.0*theta) - 0.5*g*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*g*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*f*cos(q2 - 1.0*q1 + q3 + theta) - 1.28e-12*f*sin(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*sin(q2 - 1.0*q1 + q3 + theta), -1.0e-33*(5.0e32*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 5.0e32*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57))*(g + h), 0], [a*cos(q1 - 1.0*theta) + 1.0e-49*h*(5.0e48*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 6.12e32*sin(q1 - 1.0*theta) + 5.0e48*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57)) + 1.0e-49*f*cos(q3)*(1.0e49*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 6.12e32*sin(q1 - 1.0*theta)*sin(q2 - 1.57)) - 1.0e-49*f*sin(q3)*(1.0e49*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 6.12e32*sin(q1 - 1.0*theta)*cos(q2 - 1.57)) + 1.0*g*cos(q4)*(0.5*cos(q1 + q2 + q3 - 1.0*theta - 1.57) + 0.5*cos(q2 - 1.0*q1 + q3 + theta - 1.57)) + e*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 6.12e-17*e*sin(q1 - 1.0*theta)*sin(q2 - 1.57) - 1.0*g*sin(q4)*(0.5*sin(q1 + q2 + q3 - 1.0*theta - 1.57) + 0.5*sin(q2 - 1.0*q1 + q3 + theta - 1.57)),   0.5*h*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 1.0e-49*f*cos(q3)*(6.12e32*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 1.0e49*sin(q1 - 1.0*theta)*sin(q2 - 1.57)) - 1.0e-49*f*sin(q3)*(6.12e32*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 1.0e49*sin(q1 - 1.0*theta)*cos(q2 - 1.57)) + 1.0*g*cos(q4)*(0.5*cos(q1 + q2 + q3 - 1.0*theta - 1.57) - 0.5*cos(q2 - 1.0*q1 + q3 + theta - 1.57)) + 6.12e-17*e*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 1.0*e*sin(q1 - 1.0*theta)*sin(q2 - 1.57) - 1.0*g*sin(q4)*(0.5*sin(q1 + q2 + q3 - 1.0*theta - 1.57) - 0.5*sin(q2 - 1.0*q1 + q3 + theta - 1.57)), 0.5*g*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*g*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*h*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 1.28e-12*f*cos(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*cos(q2 - 1.0*q1 + q3 + theta) + 0.5*f*sin(q1 + q2 + q3 - 1.0*theta) - 0.5*f*sin(q2 - 1.0*q1 + q3 + theta), -1.0*(1.0e-49*g + 1.0e-49*h)*(5.0e48*cos(q2 - 1.0*q1 + q3 + q4 + 1.0*theta - 1.57) - 5.0e48*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57)), 0], [ 0, - 1.0*g*cos(q2 + q3 + q4 - 1.57) - 1.0*h*cos(q2 + q3 + q4 - 1.57) - 2.56e-12*e*cos(q2) - 1.0*e*sin(q2) - 1.0*f*cos(q2 + q3 - 1.57), - 1.0*g*cos(q2 + q3 + q4 - 1.57) - 1.0*h*cos(q2 + q3 + q4 - 1.57) - 1.0*f*cos(q2 + q3 - 1.57), -1.0*cos(q2 + q3 + q4 - 1.57)*(g + h), 0], [0, -1.0*sin(q1 - 1.0*theta), -1.0*sin(q1 - 1.0*theta), -1.0*sin(q1 - 1.0*theta), 0.5*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 6.12e-17*sin(q1 - 1.0*theta) + 0.5*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57)], [0, cos(q1 - 1.0*theta), cos(q1 - 1.0*theta), cos(q1 - 1.0*theta), 0.5*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 6.12e-17*cos(q1 - 1.0*theta)], [ 1.0, 0, 0, 0, 1.0*sin(q2 + q3 + q4 - 1.57)]])


	vel_kin_combined = np.matrix([[- 1.0*q2d*(0.5*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 1.0e-49*f*cos(q3)*(1.0e49*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 6.12e32*sin(q1 - 1.0*theta)*cos(q2 - 1.57)) + 1.0e-49*f*sin(q3)*(1.0e49*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 6.12e32*sin(q1 - 1.0*theta)*sin(q2 - 1.57)) + 1.0*g*sin(q4)*(0.5*cos(q1 + q2 + q3 - 1.0*theta - 1.57) + 0.5*cos(q2 - 1.0*q1 + q3 + theta - 1.57)) + 1.0*e*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 6.12e-17*e*sin(q1 - 1.0*theta)*cos(q2 - 1.57) + 1.0*g*cos(q4)*(0.5*sin(q1 + q2 + q3 - 1.0*theta - 1.57) + 0.5*sin(q2 - 1.0*q1 + q3 + theta - 1.57))) - 1.0*q1d*(6.12e-17*h*cos(q1 - 1.0*theta) + 1.0*a*sin(q1 - 1.0*theta) - 0.5*h*sin(q2 - 1.0*q1 + q3 + q4 + 1.0*theta - 1.57) + 0.5*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 1.0e-49*f*cos(q3)*(6.12e32*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 1.0e49*sin(q1 - 1.0*theta)*cos(q2 - 1.57)) + 1.0e-49*f*sin(q3)*(6.12e32*cos(q1 - 1.0*theta)*cos(q2 - 1.57) - 1.0e49*sin(q1 - 1.0*theta)*sin(q2 - 1.57)) + 1.0*g*sin(q4)*(0.5*cos(q1 + q2 + q3 - 1.0*theta - 1.57) - 0.5*cos(q2 - 1.0*q1 + q3 + theta - 1.57)) + 6.12e-17*e*cos(q1 - 1.0*theta)*sin(q2 - 1.57) + 1.0*e*sin(q1 - 1.0*theta)*cos(q2 - 1.57) + 1.0*g*cos(q4)*(0.5*sin(q1 + q2 + q3 - 1.0*theta - 1.57) - 0.5*sin(q2 - 1.0*q1 + q3 + theta - 1.57))) - 1.0*q3d*(0.5*g*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*g*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*f*cos(q1 + q2 + q3 - 1.0*theta) - 0.5*f*cos(q2 - 1.0*q1 + q3 + theta) + 1.28e-12*f*sin(q1 + q2 + q3 - 1.0*theta) + 1.28e-12*f*sin(q2 - 1.0*q1 + q3 + theta)) - 1.0e-33*q4d*(5.0e32*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 5.0e32*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57))*(g + h)], [0.5*g*q1d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*g*q1d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*g*q2d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*g*q2d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*g*q3d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*g*q3d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*g*q4d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*g*q4d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*h*q1d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*h*q1d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*h*q2d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*q2d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*h*q3d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*q3d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*h*q4d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*h*q4d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 1.28e-12*f*q1d*cos(q1 + q2 + q3 - 1.0*theta) + 1.28e-12*f*q1d*cos(q2 - 1.0*q1 + q3 + theta) + 1.28e-12*f*q2d*cos(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*q2d*cos(q2 - 1.0*q1 + q3 + theta) + 1.28e-12*f*q3d*cos(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*q3d*cos(q2 - 1.0*q1 + q3 + theta) + 0.5*f*q1d*sin(q1 + q2 + q3 - 1.0*theta) + 0.5*f*q1d*sin(q2 - 1.0*q1 + q3 + theta) + 0.5*f*q2d*sin(q1 + q2 + q3 - 1.0*theta) - 0.5*f*q2d*sin(q2 - 1.0*q1 + q3 + theta) + 0.5*f*q3d*sin(q1 + q2 + q3 - 1.0*theta) - 0.5*f*q3d*sin(q2 - 1.0*q1 + q3 + theta) + a*q1d*cos(q1 - 1.0*theta) - 6.12e-17*h*q1d*sin(q1 - 1.0*theta) + 1.28e-12*e*q1d*cos(q1 + q2 - 1.0*theta) + 1.28e-12*e*q1d*cos(q2 - 1.0*q1 + theta) + 1.28e-12*e*q2d*cos(q1 + q2 - 1.0*theta) - 1.28e-12*e*q2d*cos(q2 - 1.0*q1 + theta) + 0.5*e*q1d*sin(q1 + q2 - 1.0*theta) + 0.5*e*q1d*sin(q2 - 1.0*q1 + theta) + 0.5*e*q2d*sin(q1 + q2 - 1.0*theta) - 0.5*e*q2d*sin(q2 - 1.0*q1 + theta)], [ - 1.0*q2d*(1.0*g*cos(q2 + q3 + q4 - 1.57) + 1.0*h*cos(q2 + q3 + q4 - 1.57) + 2.56e-12*e*cos(q2) + 1.0*e*sin(q2) + 1.0*f*cos(q2 + q3 - 1.57)) - 1.0*q3d*(1.0*g*cos(q2 + q3 + q4 - 1.57) + 1.0*h*cos(q2 + q3 + q4 - 1.57) + 1.0*f*cos(q2 + q3 - 1.57)) - 1.0*q4d*cos(q2 + q3 + q4 - 1.57)*(g + h)], [0.5*q5d*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 1.0*q3d*sin(q1 - 1.0*theta) - 1.0*q4d*sin(q1 - 1.0*theta) - 6.12e-17*q5d*sin(q1 - 1.0*theta) - 1.0*q2d*sin(q1 - 1.0*theta) + 0.5*q5d*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57)], [q2d*cos(q1 - 1.0*theta) + q3d*cos(q1 - 1.0*theta) + q4d*cos(q1 - 1.0*theta) + 6.12e-17*q5d*cos(q1 - 1.0*theta) + 0.5*q5d*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*q5d*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57)], [q1d + 6.12e-17*q2d + 6.12e-17*q3d + 6.12e-17*q4d + 3.75e-33*q5d - 1.0*q5d*sin(q2 + q3 + q4 - 1.57)]])


	combined_Torque = np.matrix([[mz + 0.5*fy*g*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*fy*g*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*fy*h*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*fy*h*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*fx*g*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*fx*g*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*fx*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 0.5*fx*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*f*fx*cos(q1 + q2 + q3 - 1.0*theta) - 0.5*f*fx*cos(q2 - 1.0*q1 + q3 + theta) + 1.28e-12*f*fy*cos(q1 + q2 + q3 - 1.0*theta) + 1.28e-12*f*fy*cos(q2 - 1.0*q1 + q3 + theta) - 1.28e-12*f*fx*sin(q1 + q2 + q3 - 1.0*theta) + 1.28e-12*f*fx*sin(q2 - 1.0*q1 + q3 + theta) + 0.5*f*fy*sin(q1 + q2 + q3 - 1.0*theta) + 0.5*f*fy*sin(q2 - 1.0*q1 + q3 + theta) + a*fy*cos(q1 - 1.0*theta) - 6.12e-17*fx*h*cos(q1 - 1.0*theta) - 1.0*a*fx*sin(q1 - 1.0*theta) - 6.12e-17*fy*h*sin(q1 - 1.0*theta) + 0.5*e*fx*cos(q1 + q2 - 1.0*theta) - 0.5*e*fx*cos(q2 - 1.0*q1 + theta) + 1.28e-12*e*fy*cos(q1 + q2 - 1.0*theta) + 1.28e-12*e*fy*cos(q2 - 1.0*q1 + theta) - 1.28e-12*e*fx*sin(q1 + q2 - 1.0*theta) + 1.28e-12*e*fx*sin(q2 - 1.0*q1 + theta) + 0.5*e*fy*sin(q1 + q2 - 1.0*theta) + 0.5*e*fy*sin(q2 - 1.0*q1 + theta)], [6.12e-17*mz + my*cos(q1 - 1.0*theta) - 1.0*mx*sin(q1 - 1.0*theta) - 1.0*e*fz*sin(q2) + 0.5*fy*g*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fy*g*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*fy*h*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fy*h*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*fx*g*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fx*g*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*fx*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fx*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 2.56e-12*fz*g*cos(q2 + q3 + q4) - 2.56e-12*fz*h*cos(q2 + q3 + q4) - 1.0*fz*g*sin(q2 + q3 + q4) - 1.0*fz*h*sin(q2 + q3 + q4) + 0.5*f*fx*cos(q1 + q2 + q3 - 1.0*theta) + 0.5*f*fx*cos(q2 - 1.0*q1 + q3 + theta) + 1.28e-12*f*fy*cos(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*fy*cos(q2 - 1.0*q1 + q3 + theta) - 1.28e-12*f*fx*sin(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*fx*sin(q2 - 1.0*q1 + q3 + theta) + 0.5*f*fy*sin(q1 + q2 + q3 - 1.0*theta) - 0.5*f*fy*sin(q2 - 1.0*q1 + q3 + theta) - 2.56e-12*f*fz*cos(q2 + q3) - 1.0*f*fz*sin(q2 + q3) + 0.5*e*fx*cos(q1 + q2 - 1.0*theta) + 0.5*e*fx*cos(q2 - 1.0*q1 + theta) + 1.28e-12*e*fy*cos(q1 + q2 - 1.0*theta) - 1.28e-12*e*fy*cos(q2 - 1.0*q1 + theta) - 1.28e-12*e*fx*sin(q1 + q2 - 1.0*theta) - 1.28e-12*e*fx*sin(q2 - 1.0*q1 + theta) + 0.5*e*fy*sin(q1 + q2 - 1.0*theta) - 0.5*e*fy*sin(q2 - 1.0*q1 + theta) - 2.56e-12*e*fz*cos(q2)], [6.12e-17*mz + my*cos(q1 - 1.0*theta) - 1.0*mx*sin(q1 - 1.0*theta) + 0.5*fy*g*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fy*g*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 0.5*fy*h*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fy*h*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*fx*g*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fx*g*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 0.5*fx*h*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*fx*h*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) - 2.56e-12*fz*g*cos(q2 + q3 + q4) - 2.56e-12*fz*h*cos(q2 + q3 + q4) - 1.0*fz*g*sin(q2 + q3 + q4) - 1.0*fz*h*sin(q2 + q3 + q4) + 0.5*f*fx*cos(q1 + q2 + q3 - 1.0*theta) + 0.5*f*fx*cos(q2 - 1.0*q1 + q3 + theta) + 1.28e-12*f*fy*cos(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*fy*cos(q2 - 1.0*q1 + q3 + theta) - 1.28e-12*f*fx*sin(q1 + q2 + q3 - 1.0*theta) - 1.28e-12*f*fx*sin(q2 - 1.0*q1 + q3 + theta) + 0.5*f*fy*sin(q1 + q2 + q3 - 1.0*theta) - 0.5*f*fy*sin(q2 - 1.0*q1 + q3 + theta) - 2.56e-12*f*fz*cos(q2 + q3) - 1.0*f*fz*sin(q2 + q3)], [6.12e-17*mz + my*cos(q1 - 1.0*theta) - 1.0*mx*sin(q1 - 1.0*theta) - 1.0*fz*cos(q2 + q3 + q4 - 1.57)*(g + h) + 1.0e-33*fy*(g + h)*(5.0e32*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 5.0e32*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57)) - 1.0e-33*fx*(5.0e32*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) + 5.0e32*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57))*(g + h)], [mx*(0.5*cos(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 6.12e-17*sin(q1 - 1.0*theta) + 0.5*cos(q2 - 1.0*q1 + q3 + q4 + theta - 1.57)) - 1.0*mz*(1.0*sin(q2 + q3 + q4 - 1.57) - 3.75e-33) + my*(0.5*sin(q1 + q2 + q3 + q4 - 1.0*theta - 1.57) - 0.5*sin(q2 - 1.0*q1 + q3 + q4 + theta - 1.57) + 6.12e-17*cos(q1 - 1.0*theta))]])

	return combined_J, vel_kin_combined, combined_Torque
