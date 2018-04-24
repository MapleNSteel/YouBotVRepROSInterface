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
import roslib; roslib.load_manifest('youbot_interface')
from youbot_interface.msg import Floats
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

from mobile_kinematics import *

running=True
deltaTime = 10./1000
elapsedTime=0

#State-Variables
odom=Odometry()

body_name='youBot_ref'
arm_joint = ['youBotArmJoint0','youBotArmJoint1','youBotArmJoint2','youBotArmJoint3','youBotArmJoint4']
gripper_joint = ['youBotGripperJoint1','youBotGripperJoint2']
mobile_joint = ['rollingJoint_rl','rollingJoint_rr','rollingJoint_fl','rollingJoint_fr']
spherical_target = 'youBot_positionTarget'

def exit_gracefully(signum, frame):

	running = False
	global clientID
	# stop the simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

	# Before closing the connection to V-REP,
	# make sure that the last command sent out had time to arrive.
	vrep.simxGetPingTime(clientID)

	# Now close the connection to V-REP:
	vrep.simxFinish(clientID)
	print('connection closed...')

	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)	
	# restore the exit gracefully handler here	
	signal.signal(signal.SIGINT, exit_gracefully)
	
def openPort():
	global clientID
	# close any open connections
	vrep.simxFinish(-1)
	# Connect to the V-REP continuous server
	clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)
def startSim():
	global clientID, arm_joint, mobile_joint, joint_handles, gripper_handles, throttle_handles, body_handle, sphere_handle

	vrep.simxSetFloatingParameter(clientID,
	        vrep.sim_floatparam_simulation_time_step,
	        deltaTime, # specify a simulation time step
	        vrep.simx_opmode_oneshot)

	# --------------------- Start the simulation

	# start our simulation in lockstep with our code
	vrep.simxSynchronous(clientID,True)
	vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

	joint_handles = [vrep.simxGetObjectHandle(clientID,
	    name, vrep.simx_opmode_blocking)[1] for name in arm_joint]

	gripper_handles = [vrep.simxGetObjectHandle(clientID,
	    name, vrep.simx_opmode_blocking)[1] for name in gripper_joint]

	throttle_handles = [vrep.simxGetObjectHandle(clientID,
	    name, vrep.simx_opmode_blocking)[1] for name in mobile_joint]
	
	body_handle = vrep.simxGetObjectHandle(clientID,
	    body_name, vrep.simx_opmode_blocking)[1]

	sphere_handle = vrep.simxGetObjectHandle(clientID,
	    spherical_target, vrep.simx_opmode_blocking)[1]

def setArmJointAngles(armJointAngles,gripperStates):
	
	global clientID, arm_joint, mobile_joint, joint_handles, gripper_handles, throttle_handles, body_handle
	
	for i in range(0,5):
		ret=vrep.simxSetJointPosition(clientID, joint_handles[i], armJointAngles[i], vrep.simx_opmode_streaming)

	#for i in range(0,2):
	#	ret=vrep.simxSetJointPosition(clientID,
	#		gripper_handles[i],
	#		gripperStates[i],
	#		vrep.simx_opmode_streaming)

def setWheelVelocities(mobileJointSpeeds):
	
	global clientID, arm_joint, mobile_joint, joint_handles, gripper_handles, throttle_handles, body_handle

	for i in range(0,4):
		vrep.simxSetJointTargetVelocity(clientID,
			throttle_handles[i],
			mobileJointSpeeds[i],
			vrep.simx_opmode_streaming)
	
def getVehicleState():

	global clientID, arm_joint, mobile_joint, joint_handles, gripper_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime

	ret, xyz=vrep.simxGetObjectPosition(clientID, body_handle, -1, vrep.simx_opmode_blocking)
	Vxyz=vrep.simxGetObjectVelocity(clientID, body_handle, vrep.simx_opmode_blocking)
	ret, rot=vrep.simxGetObjectOrientation(clientID, body_handle, -1, vrep.simx_opmode_blocking)
	
	position=np.array(xyz)
	orientation=np.array(transforms3d.euler.euler2quat(rot[0],rot[1],rot[2]))
	velocity=Vxyz[1]
	angularVelocity=Vxyz[2]

	odom.header.stamp.secs=int(elapsedTime)
	odom.header.stamp.nsecs=int((elapsedTime-int(elapsedTime))*1e9)
	
	odom.pose.pose.position.x=position[0]
	odom.pose.pose.position.y=position[1]
	odom.pose.pose.position.z=position[2]
	
	odom.pose.pose.orientation.w=orientation[0]
	odom.pose.pose.orientation.x=orientation[1]
	odom.pose.pose.orientation.y=orientation[2]
	odom.pose.pose.orientation.z=orientation[3]

	odom.twist.twist.linear.x=velocity[0]
	odom.twist.twist.linear.y=velocity[1]
	odom.twist.twist.linear.z=velocity[2]
	
	odom.twist.twist.angular.x=angularVelocity[0]
	odom.twist.twist.angular.y=angularVelocity[1]
	odom.twist.twist.angular.z=angularVelocity[2]

	pubOdom.publish(odom)

	#print("Position:        "+str(Pose))
	#print("Orientation:     "+str(orientation))
	#print("Velocity:        "+str(velocity))
	#print("Angular Velocity:"+str(angularVelocity))
	#print("")

def initialisePose():
	global clientID, joint_names, throttle_joint, joint_handles, gripper_handles, throttle_handles, body_handle, Pose

	ret, xyz=vrep.simxGetObjectPosition(clientID, body_handle, -1, vrep.simx_opmode_blocking)
	ret, rot=vrep.simxGetObjectOrientation(clientID, body_handle, -1, vrep.simx_opmode_blocking)
	Pose=np.array([xyz[0],xyz[1],rot[1]])

	#setArmJointAngles(np.array([0,0,0,0,0]),np.array([-1,-1]))
	setWheelVelocities(np.array([0,0,0,0]))

def jointCallback(msg):

		joints=msg.array
	
		setArmJointAngles(np.array([joints[0],joints[1],joints[2],joints[3],joints[4]]),np.array([0,-1]))

def wheelCallback(msg):

		array=-np.array(msg.array)

		wheelVelocities=array
	
		setWheelVelocities(np.array([wheelVelocities[0],wheelVelocities[1],wheelVelocities[2],wheelVelocities[3]]))

def main():

	global clientID, joint_names, throttle_joint, joint_handles, throttle_handles, body_handle, pubOdom, Pose, EKF, elapsedTime, sphere_handle
	
	rospy.init_node('YoubotInterface')
	rospy.Subscriber('/Youbot/SetJointStates', Floats, jointCallback)
	rospy.Subscriber('/Youbot/SetWheelVelocities', Floats, wheelCallback)
	pubOdom = rospy.Publisher('/Youbot/Odom', Odometry, queue_size=1)

	signal.signal(signal.SIGINT, exit_gracefully)
	openPort()	

	if clientID != -1: # if we connected successfully
		startSim()
		initialisePose()
		print ('Connected to remote API server')

	while(running and not(vrep.simxGetConnectionId(clientID)==-1)):
		getVehicleState()

		elapsedTime+=deltaTime
		ret=vrep.simxSetObjectPosition(clientID, sphere_handle, body_handle, (0.0,0.0,0.2), vrep.simx_opmode_blocking)
		vrep.simxSynchronousTrigger(clientID)
if __name__=="__main__":
	main()
