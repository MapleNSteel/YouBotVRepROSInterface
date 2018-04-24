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

running=True

def exit_gracefully(signum, frame):

	running = False
	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)

	try:
		if input("\nReally quit? (y/n)> ").lower().startswith('y'):
			time.sleep(1)
			sys.exit(1)

	except KeyboardInterrupt:
		print("Ok ok, quitting")
		sys.exit(1)
	
	# restore the exit gracefully handler here	
	signal.signal(signal.SIGINT, exit_gracefully)

def main():

	rospy.init_node('Example')
	pubJoints = rospy.Publisher('/Youbot/SetJointStates', Floats, queue_size=10)
	pubWheel = rospy.Publisher('/Youbot/SetWheelVelocities', Floats, queue_size=10)

	signal.signal(signal.SIGINT, exit_gracefully)	

	r = rospy.Rate(100)
	while(running):
		pubJoints.publish(np.array([0,0,pi/2,0,0]))
		pubWheel.publish(np.array([0,0,0,0]))
		r.sleep()
if __name__=="__main__":
	main()
