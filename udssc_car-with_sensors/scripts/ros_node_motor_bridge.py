#!/usr/bin/env python

import rospy
import tf
import math
import time
import datetime
import numpy as np
from numpy.linalg import inv,pinv
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from socketIO_client import SocketIO, LoggingNamespace
from sensor_msgs.msg import Joy

#Control Tuning
#turnScalar = 4
gasScalar = .5
#reverse = 0;
shiftLast = 0
steerAdjustLast = 0

steer = 0
steerlowfactor = 2.2
steerhighfactor = 2.6
limit = .5
leftlimit  = .3

def on_connect():
	print ('connected')
def cmdZumo(index,v,w):
#Send commands to zumos
#Add paper reference
	l = 0.9
	rv = v+0.5*l*w
	lv = v-0.5*l*w
 	cur_speed = (rv + lv)/2
	if(abs(cur_speed)>.5):
		eff = .5/abs(cur_speed)
		rv = rv*eff
		lv = lv*eff
	velocities = str(index) + ':' + str(v) + ',' + str(w) + ';'# +str(reverse) +'&'
	print velocities
	socketIO.emit('getvelocity', velocities)

def callback(msg): # string contains: i,xRef,yRef
    print('check')

    cmdZumo(19,msg.data[0],-msg.data[1])

def main():
    global tfl
    rospy.Subscriber('zumo17/motor_comm', Float32MultiArray, callback)
    rospy.init_node('motorcomm')
    print("Node created")
    rospy.spin()

if __name__ == '__main__':
    global data
    global timeBefore
    global thetaBefore
    global timeStart
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    shiftLast = time.time()
    steerAdjustLast = time.time()
    main()
