import numpy as np
from numpy import pi
from numpy import tan,arctan2,sin,cos
from time import sleep
import signal
import time
import sys
import rospy
import math
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Twist
import roslib; roslib.load_manifest('youbot_interface')
import transforms3d
from KalmanFilters.EKF import ExtendedKalmanFilter
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from youbot_interface.msg import Floats

running=True
deltaTime = 50./1000

previousForwBackVel=0
previousLeftRightVel=0
previousRotVel=0

accumForw=0
accumLeft=0
accumRot=0

currentPose=np.array([0,0,0])
target=np.array([1,0,1])

ready=False

r=0.02375*2

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
def targetCallback(msg):
	global target, ready, currentPose
	
	target=msg.array

	ready=True

def odomCallback(odom):

	global currentPose

	currentPose=np.array([0.,0.,0.])

	orientation=odom.pose.pose.orientation

	angles=transforms3d.euler.quat2euler(np.array([orientation.w,orientation.x,orientation.y,orientation.z]))
	rot=np.array(transforms3d.euler.euler2mat(angles[0],angles[1],angles[2]))

	alpha=np.arctan2(rot[0,1], rot[0,2])
	beta=np.arctan2(rot[0,0], rot[0,2])
	theta=np.arctan2(-rot[2,1], -rot[2,0])
	
	currentPose[0]=odom.pose.pose.position.x
	currentPose[1]=odom.pose.pose.position.y
	currentPose[2]=theta

def controlWheels():

	global target, currentPose, reached, pubWheel,  previousForwBackVel, previousLeftRightVel, previousRotVel, ready

	if(not ready):
		return

	relativePose=target-currentPose

	KpP=20
	KpT=20

	maxV=2
	maxVRot=3
	accelF=0.035

	theta=currentPose[2]
	relativePose=target-currentPose
	relativePose[0:2]=np.array([relativePose[0]*cos(theta)+relativePose[1]*sin(theta), relativePose[0]*sin(theta)+relativePose[1]*-cos(theta)])

	if(np.linalg.norm(relativePose)<5e-2):
		pubWheel.publish(np.array([0,0,0,0]))
		ready=False

		return
	
	forwBackVel=relativePose[0]*KpP
	leftRightVel=relativePose[1]*KpP

	v=np.sqrt(forwBackVel**2+leftRightVel**2)
	if(math.isnan(v) or math.isnan(relativePose[2])):
		return

	if (v>maxV):
		forwBackVel=forwBackVel*maxV/v
		leftRightVel=leftRightVel*maxV/v

	rotVel=arctan2(sin(relativePose[2]), cos(relativePose[2]))*KpT
	if (abs(rotVel)>maxVRot):
		rotVel=maxVRot*rotVel/abs(rotVel)

	df=forwBackVel-previousForwBackVel
	ds=leftRightVel-previousLeftRightVel
	dr=rotVel-previousRotVel

	if (abs(df)>maxV*accelF):
		df=abs(df)*(maxV*accelF)/df

	if (abs(ds)>maxV*accelF):
		ds=abs(ds)*(maxV*accelF)/ds

	if (abs(dr)>maxVRot*accelF):
		dr=abs(dr)*(maxVRot*accelF)/dr

	forwBackVel=previousForwBackVel+df
	leftRightVel=previousLeftRightVel+ds
	rotVel=previousRotVel+dr

	pubWheel.publish(np.array([forwBackVel-leftRightVel-rotVel,forwBackVel+leftRightVel+rotVel,forwBackVel+leftRightVel-rotVel,forwBackVel-leftRightVel+rotVel]))

	previousForwBackVel=forwBackVel
	previousLeftRightVel=leftRightVel
	previousRotVel=rotVel
def main():
	global reached, pubWheel

	rospy.init_node('MechanimPID')
	
	rospy.Subscriber('/Youbot/SetVehicleTarget', Floats, targetCallback)
	rospy.Subscriber('/Youbot/Odom', Odometry, odomCallback)
	pubWheel = rospy.Publisher('/Youbot/SetWheelVelocities', Floats, queue_size=1)

	signal.signal(signal.SIGINT, exit_gracefully)	

	r = rospy.Rate(100)
	while(running):
		controlWheels()
		r.sleep()
if __name__=="__main__":
	main()
