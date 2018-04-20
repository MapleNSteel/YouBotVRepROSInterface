import numpy as np
from numpy import pi
from numpy import tan,arctan,sin,cos
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

running=True


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

def main():

	global running

	rospy.init_node('Example')
	pubTarget=rospy.Publisher('/Youbot/SetVehicleTarget', Floats, queue_size=10)

	signal.signal(signal.SIGINT, exit_gracefully)	

	r = rospy.Rate(100)
	while(running):
		pubTarget.publish(np.array([0,0,0]))
		r.sleep()
		
if __name__=="__main__":
	main()
