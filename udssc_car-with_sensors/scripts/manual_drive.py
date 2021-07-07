#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import numpy as np

import socket #send udp messages
import sys #read arguments

global sock, udpPort, hostname

global reverse

def parseJoy(msg):
	""" Convert Joy to motoclutch pedalr command """

#	print msg

	# Shifting
	if msg.buttons[17]:
		reverse = True
	else:
		reverse = False

	# Gas:
	upper_limit = 0.5
	gas = msg.axes[2] #gas element in [-1, 1]
	gas = (gas+1)/2   #normalize to [0, 1]


	gas = gas * upper_limit # set to [0, UL]
	# brake
	brake = msg.axes[3] #gas in [-1, 1]
	#brake if gas > -0.5
	if brake > 0.0:
		gas = 0.0
	if reverse:
		gas *= -1.0	#set default values
	#todo: the gas does nothing right now, the car should probably
	#coast when the break is not pressed rather than stopping

	# Steering:
	steering_sens = 3 #steering sensitivity
	steering_limit = 45 #max steering angle

	#read steering angle
	steering = msg.axes[0] # angle in range [-1, 1]
	steering = np.sign(steering) * np.sqrt(np.abs(steering)) * steering_limit

	cmd = str(gas) + "," + str(steering)

#	print cmd
	sendCmd(cmd)

def sendCmd(cmd):
	""" Sends command to carNode """
	print cmd
	#message creation from C++ code, 2/24/2020
	#std::string msg = header + type + ":" + pos + ":" + segment + ":" + length + ":" + paths + ":" + speeds + ":" + times + ":" + tseg;
	header = "0|0|MAN" #header is initial-cum dist.|bias|type
	pos = "0:0:0:0:0" #pos is xpos:ypos:yaw:vel:yawrate
	segment = "A0" #path ID string
	length = "0" #path length
	paths = "0:0:0:0" #current and next path x:y equations
	speeds = cmd
	times = "0,0"	#set default values
	gas = 0.0
	tseg = "0"

	msg = header + ":" + pos + ":" + segment + ":" + length + ":" + paths + ":" + speeds + ":" + times + ":" + tseg

#	print msg
	sock.sendto(msg, (hostname, udpPort))

if __name__ == "__main__":

	if len(sys.argv) == 1:
		print "Please supply the CAV hostname as the first argument"
		sys.exit()

	#set up the joystick node and subscribe
	rospy.init_node("joy_test", log_level=rospy.DEBUG)
	sub = rospy.Subscriber("/joy", Joy, parseJoy)

	#connect via UDP to the car
	udpPort = 2525
	hostname = sys.argv[1]

	print str(udpPort) + ", " + hostname

	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


	reverse = False
	print "subscriber and socket created, spinning..."

	sendCmd("0,0")

	rospy.spin()
