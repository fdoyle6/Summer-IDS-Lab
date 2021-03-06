#!/usr/bin/env python
import RPi.GPIO as GPIO
from subprocess import Popen
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)#Red
GPIO.setup(15,GPIO.OUT)#Green
GPIO.setup(16,GPIO.OUT)#Blue
import os
import platform
import rospy
import numpy as np
from scipy import linalg
import time
import compiler
import tf
import serial
import math
import atexit
from math import *
from geometry_msgs.msg import *
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud
from udssc_msgs.msg import Command
from udssc_msgs.srv import *
#For reading .csv gains
import csv

from threading import Thread, Lock

import socket

#######################################################################################
##IMPORTANT: CHANGE CODE VERSION BEFORE COMMITTING#####################################
##IF BIG CHANGE THAT IS REQUIRED ON ALL CARS, ITERATE FIRST DIGIT AND START .0 VERSION#
##IF SMALL CHANGE, ITERATE AFTER DECIMAL###############################################
#######################################################################################
global code_version
code_version = "2.5"

# Data file names - NOTE: Change N & X each run
global file1_name, file2_name, file3_name, file1, file2, file3, recordingData

recordingData = True

runNumber = '2' #Run number and car number for exported data files
carNumber = '3'

file1_name = 'data_collection/VICON_Data-Run_' + runNumber + '-Car_Number_' + carNumber +'.txt'
file2_name = 'data_collection/Sensor_Data-Run_' + runNumber + '-Car_Number_' + carNumber +'.txt'
file3_name = 'data_collection/Waypoint_Data-Run_' + runNumber + '-Car_Number_' + carNumber +'.txt'

global lf

class line_follower(object):

    def __init__(self, car_length=0.111506):

		# Stanley Controller
        self.sk = 4#0.85  #very sensitive
        self.sk_soft = 0.001
        self.sk_yaw = .1 #0.00125
        self.mass = 0.37#kg
        cy = 0.25 #N/rad
        self.sk_ag = -self.mass/(2*cy)
		
		# ROS Node Initialization	
        hostname = platform.node()
        idnum = platform.node().split('_')[1]
        node_name = hostname + "_path_controller"
        rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(50)

		# Data string from mainframe
        self.dataString = ""
        self.dataString_old = ""

		# Fetch parameters
	#	self.calib = rospy.get_param("/car_config/muskrat_" + idnum + "/steering_bias", 0)
        self.calib = 0
		#TEMP
        self.r = 0
        self.init = False


        wait_loop = rospy.Rate(30)
#		while self.r == None and not rospy.is_shutdown():
#			try:
#				self.r = rospy.get_param("/car_config/" + hostname + "/initial_distance")
#			except:
#				self.r = None
#				rospy.logdebug("==WAITING ON MAINFRAME START==")
#			wait_loop.sleep()

		# General variables
        self.hostname = hostname
        self.cmd_type = ""
        self.id = idnum
        self.path_equations = ["0","0"]
        self.next_path_equations = ["0","0"]
        self.path_length = 0
        self.segment_id = ""
        self.segment_id_old = ""
        self.path_length = 0
        self.using_next_path = False
        self.velocity_profile = "0"
        self.velocity_desired = 0
        self.t_seg_mainframe = None
        self.prev_path_length = 0
        self.time_old = 0
        self.time = 0
        self.position_old = np.zeros(3)
        self.vel_old = 0
        self.yaw_dot = 0
        self.theta_old = 0
        self.e_ahead_old = 0
        self.e_i = 0
        self.vicon_pos = [0,0,0]
        self.euler = 0
        self.vicon_speed = 0
        self.vicon_yawrate = 0
        self.pos = np.zeros((2,3))
        self.car_length = car_length
        self.t_timeout = 0
        self.tseg = 0
        self.yaw_traj_rate = 0
        self.yaw_traj_new = 0
        self.yaw_traj_old = 0
        self.t0 = rospy.get_time()
        
        self.phi = 0.0
        self.phi_dot = 0.0

		# Running variables
		# vicon and sensor data variables
        self.vX = 0.0 	#VICON Variables
        self.vY = 0.0 
        self.vTheta = 0.0 
        self.vThetaDot = 0.0 
        self.vSpeed = 0.0 
        self.vTime = rospy.get_time() 
		#speed is just to check later

        self.sVelo = 0.0 	#Sensor Variables
        self.sAccel0 = 0.0 
        self.sAccel1 = 0.0 
        self.sAccel2 = 0.0 
        self.sMag0 = 0.0 
        self.sMag1 = 0.0 
        self.sMag2 = 0.0 
        self.sGyro0 = 0.0 
        self.sGyro1 = 0.0 
        self.sGyro2 = 0.0 
        self.sTime = rospy.get_time()

        self.desiredX = 0.0	# Waypoint Variables
        self.desiredY = 0.0
        self.desiredVhead = 0.0
        self.desiredVlat = 0.0
        self.desiredTheta = 0.0
        self.desiredTheta_dot = 0.0
        self.desiredT = 0.0 

		# for old data variables
        self.o_vX = 0.0 
        self.o_vY = 0.0 
        self.o_vTheta = 0.0 
        self.o_vThetaDot = 0.0 
        self.o_vSpeed = 0.0 
        self.o_vTime = rospy.get_time()
 
        self.o_sVelo = 0.0 
        self.o_sAccel0 = 0.0 
        self.o_sAccel1 = 0.0 
        self.o_sAccel2 = 0.0 
        self.o_sMag0 = 0.0 
        self.o_sMag1 = 0.0 
        self.o_sMag2 = 0.0 
        self.o_sGyro0 = 0.0 
        self.o_sGyro1 = 0.0 
        self.o_sGyro2 = 0.0 
        self.o_sTime = rospy.get_time()

        self.o_desiredX = 0.0
        self.o_desiredY = 0.0
        self.o_desiredVhead = 0.0
        self.o_desiredVlat = 0.0
        self.o_desiredTheta = 0.0
        self.o_desiredTheta_dot = 0.0
        self.o_desiredT = 0.0
       
        self.batteryVoltage = 0.0	
        
        ''' u_in = input('Would you like to record the data from this experiment? (y/n)')
        if u_in.lower() == 'y':
            global recordingData
            recordingData = True ''' 
        
    	# comparison vectors (time not included in state vector)
        if recordingData:
            self.ViconState = np.array([ self.vX, self.vY, self.vTheta, self.vThetaDot, self.vSpeed ])
            self.sensorState = np.array([ self.sVelo, self.sAccel0, self.sAccel1, self.sAccel2, self.sMag0,
                                         self.sMag1, self.sMag2, self.sGyro0, self.sGyro1, self.sGyro2 ])
            self.wayPoint = np.array([ self.desiredX, self.desiredY, self.desiredVhead, self.desiredVlat, self.desiredTheta, 
                                      self.desiredTheta_dot ])

            self.oldViconState =  np.array([ self.o_vX, self.o_vY, self.o_vTheta, self.o_vThetaDot, self.o_vSpeed ])
            self.oldSensorState = np.array([ self.o_sVelo, self.o_sAccel0, self.o_sAccel1, self.o_sAccel2, self.o_sMag0, 
                                            self.o_sMag1, self.o_sMag2, self.o_sGyro0, self.o_sGyro1, self.o_sGyro2 ])
            self.oldWayPoint = np.array([ self.o_desiredX, self.o_desiredY, self.o_desiredVhead, self.o_desiredVlat, 
                                         self.o_desiredTheta, self.o_desiredTheta_dot ])
           
        self.sensorString = ''
        self.sensorData = ['', '']
		
        # State Estimation Variables
        self.X = np.zeros(shape = (1, 6))
        self.X_hat = np.zeros_like(self.X)
        self.X_dot = np.zeros_like(self.X)
        self.U = np.zeros(shape = (1, 3))
        self.Y_hat = np.zeros(shape = (1, 4))
        self.Y_sensor = np.zeros(shape = (1, 4))
        self.dt = 25/1000   # ~ 25 milliseconds (updates every loop)
        self.loops_without_vicon = 0
        
        # State Estimation Probability
        self.P = np.zeros(shape = (6, 6))
        self.I = np.identity(like = self.P)
        
        # Dynamical Model Matrices
        self.F = np.zeros(shape = (6, 6)); self.F[4][5] = 1  # this changes every iteration so just declare the variable
        self.B = np.array([ [0, 0, 0], # static matrix
                           [0, 0, 0],
                           [1, 0, 0],
                           [0, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1] ])
        
        self.C = np.zeros((4, 6))   # Also static so hard-coded
        self.C[0][2] = 1; self.C[1][3] = 1; self.C[2][4] = 1; self.C[3][5] = 1
        
        # Uncertainty Matrices
        # TODO - Update these Matrices
        self.V_dist = np.diag([ 1, 1, 1, 1, 1, 1])
        self.V_noise = np.diag([ 1, 1, 1, 1])
        
		#state read
        self.soc = -1
        self.position_desired = np.zeros(2)

		#mainframe stuff
        self.time_stamp = ""


		# Velocity Controller
        self.velocity_current = 0 #Vicon velocity
        self.last_vicon_t = 0
        self.phi_r = 0
        self.phi_c = 0
        self.mainframe_mutex = Lock()
        self.vicon_mutex = Lock()
        self.e_head = 0

		# Serial
        if hostname[:5]=="manta":
            self.ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0)
        else:
            self.ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=0)
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.write("0,0;")

    	# ROS Topics
        self.update_pose = rospy.Publisher(self.hostname + '/pose', PoseStamped, queue_size=5)
        self.update_target = rospy.Publisher(self.hostname+'/target', PoseStamped, queue_size=5)
        self.update_steering = rospy.Publisher(self.hostname+'/steering', PoseStamped, queue_size=5)

		# Velocity Control
        self.antiwindup = 0.0125#0.0005 #maximum integral value
        self.k_p = 0.1225#0.0255
        self.k_i = 0.015
        
		#saved control parameters
        self.pos_err_i = 0
        self.last_pos_err = 0
		
        # SMC
		# s = y_eDot + k*y_e + k0*sign(y_e)*theta_e
        self.Q = 1.0 #num, multiplies s {1}
        self.P = 1.0  #num, multiplies sign(s) {1}
        self.k = 0.5 #num, multiplies vr*sin(theta_e) change it very small, (sensitive) {.5}
        self.k_0 = 0.05 #den, multiplies sign(y_e) {.05}
        self.threshold = 0.075#{.075} smoothing threshold for sign functions
        self.Lh = 0.00 #lookahead distance in m
        self.sk_steer = 0.1
        self.prev_steer = 0

		#Manual Driven
        self.manual_speed = 0
        self.manual_steer = 0
        self.cameraOn = False

        self.steering_bias = 0

		#Default values for Manta
        if hostname[:5]=="manta":
            self.sk = 6.0 #7.0
            self.sk_soft = 0.1 #.001
            self.sk_yaw = .1
            self.sk_steer = 0.1
            self.mass = .364
            self.car_length = .18
            cy = .8 * 2
            self.sk_ag = -self.mass/(2*cy)
            self.steering_bias = 0
            self.Lh = 0.00
			
		#Read CSV to overwrite gains if necessary
        with open('/home/pi2/catkin_ws/src/udssc_car/scripts/gains.csv', mode='r') as csvfile:
            gainsreader = csv.DictReader(csvfile)
            for row in gainsreader:
                if row["name"]==hostname:
                    self.sk = float(row["sk"])
                    self.sk_soft = float(row["sk_soft"])
                    self.sk_yaw = float(row["sk_yaw"])
                    self.mass = float(row["mass"])
                    cy = float(row["cy"])
                    self.car_length = float(row["car_length"])
                    #Recalculate the following values:
                    self.sk_ag = -self.mass/(2*cy)
        self.RGB = "010"


		# Create files if collecting data
        print(recordingData)
        if recordingData:
            global file1, file2, file3;
            file1 = open(file1_name, "w")         # file for VICON data
            file2 = open(file2_name, "w")         # file for sensor data
            file3 = open(file3_name, "w")	   # file for idealized trajectories

    		# Add headers to data files
            saveData(file1, 'Time', ['X', 'Y', 'Heading_Angle', 'Angular_Velocity', 'Speed'])
            saveData(file2, 'Time', ['Heading_Velocity', 'Acceleration_1', 'Acceleration_2', 
                                     'Acceleration_3', 'Compass_1', 'Compass_2', 'Compass_3', 
                                     'Gyro_1', 'Gyro_2', 'Gyro_3'])
            saveData(file3, 'Time', ['Desired_X', 'Desired_Y', 'Desired_V_head', 
                                             'Desired_V_lat', 'Desired_Theta', 'Desired_Theta_dot'])

		# 2 components of acceleration, 2 compass components, and 2 gyro components are useless 
		# for right now, but I'm keeping all of them so I know how the sensors work fully.
		
        rospy.loginfo("Initialization complete!")
        rospy.loginfo("Using code version: " + code_version)
        time.sleep(1)


    def reset(self):
        self.dataString_old = ""
        self.calib = 0
        self.r = 0
        self.init = False
        self.path_equations = ["0","0"]
        self.next_path_equations = ["0","0"]
        self.path_length = 0
        self.segment_id = ""
        self.segment_id_old = ""
        self.path_length = 0
        self.using_next_path = False
        self.velocity_profile = "0"
        self.velocity_desired = 0
        self.t_seg_mainframe = None
        self.prev_path_length = 0
        self.time_old = 0
        self.time = 0
        self.position_old = np.zeros(3)
        self.vel_old = 0
        self.yaw_dot = 0
        self.theta_old = 0
        self.e_ahead_old = 0
        self.e_i = 0
        self.vicon_pos = [0,0,0]
        self.euler = 0
        self.vicon_speed = 0
        self.vicon_yawrate = 0
        self.pos = np.zeros((2,3))
        self.t_timeout = 0
        self.tseg = 0
        self.yaw_traj_rate = 0
        self.yaw_traj_new = 0
        self.yaw_traj_old = 0
        self.position_desired = np.zeros(2)
        self.time_stamp = ""
        self.velocity_current = 0 
        self.last_vicon_t = 0
        self.phi_r = 0
        self.phi_c = 0
        self.e_head = 0
	

    def addInRange(self, val, add, minval=(-30), maxval=(30)):
        """ Adds within a specified range """

        newval = val + add
        if newval < minval: return minval
        if newval > maxval: return maxval
        return newval

    def broadcast(self, steering_angle,pos_d,dR_d):
		# Broadcast target
        pose_msg = PoseStamped()
		#TODO: Increment this part
        pose_msg.header.seq = 0
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        point = Point()
        point.x = self.pos[0,0]
        point.y = self.pos[0,1]
        point.z = self.pos[0,2]
        orientation = Quaternion()
        theta_steering = self.pos[1,2] + np.deg2rad(steering_angle)
        quaternion = tf.transformations.quaternion_from_euler(0,0,theta_steering)
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        pose_msg.pose.position = point
        pose_msg.pose.orientation = orientation
        self.update_steering.publish(pose_msg)

		#send the desired position message
		#TEMP: Send to its own function
		# Broadcast target
        pose_msg = PoseStamped()
		#TODO: Increment ttransform = tf_buffer.lookup_transform(target_frame,his part
        pose_msg.header.seq = 0
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        point = Point()
        point.x = pos_d[0]
        point.y = pos_d[1]
        point.z = 0.115459
        orientation = Quaternion()
        theta_path = np.arctan2(dR_d[1],dR_d[0])
        quaternion = tf.transformations.quaternion_from_euler(0,0,theta_path)
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        pose_msg.pose.position = point
        pose_msg.pose.orientation = orientation
        self.update_target.publish(pose_msg)



    def calcError(self):
        """ Calculates error for use in the Stanley controller """


		#self.Lh = 0.0
        pos_d, dR = self.evalPath(self.r + self.Lh)

        if self.pos[0][0] == 0 and self.pos[0][0] == 0:
            return 0, 0

        x_d = pos_d[0]
        y_d = pos_d[1]
        theta_d = atan2(dR[1], dR[0])
        theta_r = self.pos[1][2]


        dx = self.pos[0][0] - x_d
        dy = self.pos[0][1] - y_d

        y_e = dx*( -sin(theta_d) ) + dy *( cos(theta_d) )

        theta_e = theta_r - theta_d
        if theta_e > math.pi:
            theta_e = theta_e - 2*pi
        if theta_e < -math.pi:
            theta_e = 2*math.pi + theta_e

        return -y_e, -theta_e



	#### PARSE THE MAINFRAME MESSAGE ####
    def manualDrive(self):
        if(self.cameraOn == False):
            self.cameraOn = True
            rc = Popen("/home/pi2/catkin_ws/src/udssc_car/scripts/cameraBash.sh")
        cmd = str(self.manual_steer) + "," + str(self.manual_speed) + ';'
        print(cmd)
        self.ser.write(cmd)
		


    def msgParser(self, msg):

        self.segment_id_old = self.segment_id

        tokens = msg.split(':')
        msg_type= tokens[0]
        x_pos   = tokens[1]
        y_pos   = tokens[2]
        theta   = tokens[3]
        speed   = tokens[4]
        yawrate = tokens[5]
        segment = tokens[6]
        length  = tokens[7]
        f_x     = tokens[8]
        f_y     = tokens[9]
        f_x2    = tokens[10]
        f_y2    = tokens[11]
        speeds  = tokens[12]
        times   = tokens[13]
        t_seg   = tokens[14]

#		print "##################################"
#		print "type:   \t" + msg_type
#		print "xpos:   \t" + x_pos
#		print "ypos:   \t" + y_pos
#		print "theta:  \t" + theta
#		print "segment:\t" + segment
#		print "length: \t" + length
#		print "f_x:    \t" + f_x
#		print "f_y:    \t" + f_y
#		print "f_x2:   \t" + f_x2
#		print "f_y2:   \t" + f_y2
#		print "speeds: \t" + speeds
#		print "times:  \t" + times
#		print "tseg:   \t" + t_seg
        speeds_string = speeds.split(',')
        times_string = times.split(',')
        time_stamp =[]
        velocity_profile =[]
        for i in range(len(speeds_string)):
            time_stamp.append (float(times_string[i]))
            velocity_profile.append( float(speeds_string[i]))

        self.segment_id = segment

        if msg_type == "IDM":
            self.r = max(float(length), 0.01)
            length = "100"
        else:
            if not (self.segment_id == self.segment_id_old):
                self.r -= self.path_length
                if self.r < 0:
                    self.r = 0
        if msg_type == "MAN":
            self.manual_speed = float(speeds_string[0])
            self.manual_steer = float(speeds_string[1])
        
        self.cmd_type = msg_type
        self.time_stamp = time_stamp
        self.path_length = length
        self.velocity_profile = velocity_profile
        self.path_length = float(length)
        self.path_equations[0] = f_x
        self.path_equations[1] = f_y
        self.next_path_equations[0] = f_x2
        self.next_path_equations[1] = f_y2
        self.vicon_pos[0] = float(x_pos) 
        self.vicon_pos[1] = float(y_pos)
        self.vicon_speed = float(speed)
        self.vicon_yawrate = float(yawrate)
        self.euler = float(theta)
        self.t = rospy.get_time()
        self.tseg = float(t_seg)





#### MAIN UPDATE LOOP ####

    def update(self):
        GPIO.output(18,int(self.RGB[0]))
        GPIO.output(16,int(self.RGB[1]))
        GPIO.output(15,int(self.RGB[2]))
		#print self.r, "/", self.path_length, " on ", self.segment_id, "\tSOC:", self.soc


        ''' Main update function '''
        #TODO USE CHRONY TO SYNC clock (Easy)
        self.time = rospy.get_time()
        if (self.time_old == 0):
            self.time_old = rospy.get_time()
		
        mf_commands = self.dataString

        if self.dataString == "END":
            self.ser.write(str(0) + ',-' + str(0) + ';')
            return

        if (mf_commands != self.dataString_old):
			# interpret the commands (recieved VICON Update) & update the state variables            
            self.msgParser(mf_commands)
            self.P = np.zeros_like(self.P)   # Uncertainty of being at the vicon state is 0
            self.loops_without_vicon = 0
            
            self.X[0] = self.vicon_pos[0]; self.X[1] = self.vicon_pos[1]
            self.X[2] = self.vicon_speed*np.cos(self.phi); self.X[3] = self.vicon_speed*np.sin(self.phi)
            self.X[4] = self.euler; self.X[5] = self.vicon_yawrate
            
            self.updateF()
        else:
            self.estimateState()
            
            self.loops_without_vicon += 1
            
            self.vicon_pos[0] = self.X[0]; self.vicon_pos[1] = self.X[1]
            self.vicon_speed = np.abs(self.X[2] + 1j*self.X[3])
            self.euler = self.X[4]; self.vicon_yawrate = self.X[5]
            

        if (self.cmd_type == "MAN"):
            #print "Manual Driving @ " + str(self.rate)  
            self.manualDrive()
            self.getSOC()
            self.rate.sleep()
            return
        
        #self.vicon_mutex.acquire()
        self.pos[0][0] = self.vicon_pos[0]
        self.pos[0][1] = self.vicon_pos[1] 
        self.pos[1][2] = self.euler
        self.velocity_current = self.vicon_speed
        self.yaw_dot = self.vicon_yawrate
        #self.vicon_mutex.release()
	
        self.desiredUpdate()
        pos_d, dR_d = self.evalPath(self.r)
        self.yaw_traj_new = atan2(dR_d[1], dR_d[0]);
        self.position_desired = pos_d

    	# get lateral and orientation error
        y_e, theta_e = self.calcError()
		
		#calculate desired yaw rate
        dt = self.time - self.time_old; self.dt = dt
        dr = (self.velocity_desired + self.velocity_current)/2 * dt
        pos_next, dR_next = self.evalPath(self.r + dr) 
        yaw_next = atan2(dR_next[1], dR_next[0])
        dydt_des = (yaw_next - atan2(dR_d[1], dR_d[0]))/dt
        yaw_traj_rate = (self.yaw_traj_new -self.yaw_traj_old)/dt


		# Steering controllers
        steering_angle = self.stanleyControl(theta_e, self.velocity_desired, y_e, dydt_des)        

        print("################################################")
        print("STEER ANGLE\t" + str(steering_angle * 180 / math.pi))
        print("LATERAL ERR\t" + str(y_e))
        print("THETA ERROR\t" + str(theta_e * 180 / math.pi))


		#convert to degrees
        steering_angle = steering_angle * 180 / math.pi
		# Add the bias term and saturate to +- 45 degrees
        steering_angle = self.addInRange(steering_angle, self.calib, -45,45)

        ctrl_velocity, direction = self.velocityController()

		# If code is slow, optimize so we don't eval path twice in v_ctrl
        if self.cmd_type == "IDM":
            velocity = self.velocity_desired
        else:
            velocity = ctrl_velocity + self.velocity_desired

        if  self.velocity_desired <= 0.001:
            velocity = 0.0
            steering_angle = 0.0
            
            
        # Update theta-double-dot in U
        d_phi = steering_angle - self.phi
        self.phi_dot = d_phi/dt
        self.U[2] = self.X_dot[2]*np.tan(self.phi)/(self.car_length) + self.X[2]*self.phi_dot/(self.car_length*(np.cos(self.phi)**2))
        
        # Update theta-dot in U
        self.phi = steering_angle
        self.U[1] = self.X[2]*np.tan(self.phi)/(self.car_length) # uses bike model
	
		# Create the serial message to the arduino and write
        motor_comm = [float(steering_angle),float(velocity)]
	
        print(velocity, "\t*********************")

        if abs(velocity) < 0.01:
#			print "TRYING TO STOP!!!!"
#			motor_comm = [steering_angle, 0]
            self.pos_err_i = 0

        if direction < 0:
#			print "POINT IS BEHIND US!!!!###########%%%%%%%%^^^^^^^$$$#\n\n"
#			motor_comm = [steering_angle,0]
            self.pos_err_i = 0

        self.ser.write(str(motor_comm[0]) + ',' + str(motor_comm[1]) + ';')
        
        if recordingData:
            self.saveFullData()

		# Store old variables
        self.time_old = self.time
        self.prev_steer = steering_angle
        self.yaw_traj_old = self.yaw_traj_new

        self.broadcast(steering_angle,pos_d,dR_d)

        self.getSOC()
        self.rate.sleep()


    def evalPath(self, r, h = 10**(-3)):
        ''' Evaluate path equation using a given distance along said path.
		Returns position and tangent '''
#		self.mainframe_mutex.acquire()
        if r < 0:
            rospy.logerr("ERROR: Path length less than zero " + str(r))
            r = .01 #temporary fix

        if r <= self.path_length:
            pathX = self.path_equations[0]
            pathY = self.path_equations[1] 
        else:
            r = r - self.path_length
            pathX = self.next_path_equations[0]
            pathY = self.next_path_equations[1]
#		self.mainframe_mutex.release()
        result = np.array((eval(pathX),eval(pathY))) # Resulting position from evaluating path at distance r
	
        r_path = r

        r = r_path + h
        f1 = np.array((eval(pathX),eval(pathY)))
        r = r_path - h
        f0 = np.array((eval(pathX),eval(pathY)))
        d = (f1 - f0)/(2 * h)

        yaw = atan2(d[1], d[0])

        return result, d


    def getSOC(self):
        if self.ser.in_waiting:
            try:
                soc = self.ser.readline()
                self.soc = float(soc)
                self.ser.flushInput()
	
                if float(soc) < 7.25:
                    print("\033[1;31mWARNING! LOW SOC OF " + soc +"\033[0m")
                    RGB = "111"

            except ValueError:
                print("error with SOC reading:\t " + soc)



    def desiredUpdate(self):

        self.vel_old = self.velocity_desired

		### CALCULATE DESIRED VELOCITY###   ####TODO figure this out

        time_stamp = self.time_stamp
        velocity_profile = self.velocity_profile

        experiment_time = (rospy.get_time() - lf.t0)


        DESIRED_SPEED = 0

        if self.cmd_type == "IDM":
            DESIRED_SPEED = velocity_profile[0]
        else:
            for i in range(len(time_stamp)-1):
                if experiment_time > time_stamp[i] and experiment_time < time_stamp[i+1]:
                    d_time = time_stamp[i+1] - time_stamp[i]
                    d_velocity = velocity_profile[i+1] - velocity_profile[i]
                    acceleration = d_velocity/d_time
                    self.U[0] = acceleration
                    DESIRED_SPEED = velocity_profile[i] + acceleration * (experiment_time - time_stamp[i])

#		print str(DESIRED_SPEED) + " @@@@ " + str(experiment_time)
		### SAVE DESIRED VELOCITY ###
		#todo do the entire update here rather than in the mainframe cb

        self.velocity_desired = DESIRED_SPEED

		### INTEGRATE POSITION WITH TRAPEZOIDAL RULE ###
        if self.cmd_type == "TRAJ":
            print("self.r="+str(self.r) +"+("+str(self.vel_old)+"+"+str(self.velocity_desired)+")*("+str(self.time)+"-"+str(self.time_old)+")/2")
            self.r = self.r + (self.vel_old + self.velocity_desired)*(self.time-self.time_old)/2




    def shutdown(self):
        """ Turn off motor on shutdown """
        self.ser.write("0,0;")
        self.ser.close()
        global file1, file2, file3
        file1.close(); file2.close(); file3.close()
        exit()



    def stanleyControl(self, psi, SpeedDesired, y_e, yaw_dot_desired):
        num = (self.sk * y_e)
        den = (self.sk_soft + self.velocity_current)
        steer =  (psi - self.sk_ag * self.velocity_current * yaw_dot_desired) + atan2(num,den) - self.sk_yaw*(self.yaw_dot - yaw_dot_desired)

        steer_final = steer + self.sk_steer * (steer - self.prev_steer * 3.14 / 180) #steering is converted to degrees outside of this function somewhere...
        
        print( "*****STANLEY*****" )
        print( "Psi Value\t" + str(psi*180/3.14))
        print( "Sk_ag term\t"+str(-self.sk_ag*SpeedDesired*yaw_dot_desired*180/3.14))
        print( "Sk/sk_soft\t"+str(atan2(num,den)*180/3.14) )
        print( "Sk_yaw term\t"+str(self.sk_yaw*(self.yaw_dot - yaw_dot_desired)*180/3.14) )
        print( "Steer Result\t" + str(steer_final*180/3.14) )
        print( "****************" )
        print( "SpeedDesired\t" + str(SpeedDesired) )
        print( "Yaw_Dot_desired\t" + str(yaw_dot_desired*180/3.14) )
        print( "steer-prev steer\t"+str((steer - (self.prev_steer*3.14/180))*180/3.14) )
        print( "****************" )

        return steer_final

    def timeoutCheck(self, timeout=1):
        """ Checks for timeout based on timestamp from mainframe messages """
        if (self.time - self.t_timeout) > timeout:
            return True
        else:
            return False



    def updatePath(self, msg):
        """ Update the path based on information from the mainframe """

        if msg.type.lower() != "traj":
            return




    def velocityController(self):
        """ Velocity Controller based on distance to target """
        target_position, d = self.evalPath(self.r)
        # Calculate car normal
        car_heading = self.pos[1,2]

		# Position error in cars local coordinates
        position_error = target_position - self.pos[0,:2]
        position_error_magnitude = abs(np.linalg.norm(position_error))
        print( "Position Error is " + str(position_error) + "MAGN. : " + str(position_error_magnitude) + " R:" +str(self.r) )

        if self.pos[0][0] == 0 and self.pos[0][1] == 0:
            return 0, 0

		# Determine direction
        theta_positions = np.arctan2(position_error[1], position_error[0])
        theta_diff = (theta_positions+np.pi) - (car_heading+np.pi)

		# Fix heading ie: -179deg and 179deg are only 2 degrees apart not 358
        if abs(theta_diff) > np.pi:
            theta_diff = 2*np.pi - abs(theta_diff)

        if abs(theta_diff) > 0.5*np.pi:
            direction = -1
        else:
            direction = 1

        if target_position[0] == 0 and target_position[1] == 0:
            direction = 0

        pos_err = direction * position_error_magnitude

        self.pos_err_i = (pos_err + self.last_pos_err)/2 * (self.time - self.time_old)
        self.pos_err_i = self.addInRange(self.pos_err_i, 0, -self.antiwindup, self.antiwindup)

        v_cmd = pos_err*self.k_p + self.pos_err_i*self.k_i

        self.last_pos_err = pos_err

        return v_cmd, direction
    
    
    
    def estimateState(self):
        '''Reads Sensors & Updates State if no reading from VICON'''
        
        # Poll Arduino for Sensor Data
        self.ser.write('1')
        
        # Predict the current X
        self.X_dot = (self.F @ self.X) + (self.B @ self.U)
        self.X_hat = self.X + self.X_dot * self.dt
        
        # Update Probability (A Priori)
        self.P = (self.F @ self.P) @ self.F.transpose() + self.V_dist
        
        # uses Y_i = C X_i ; X_dot_i = F_i X_i + B U_i
        self.Y_hat = self.C @ self.X_hat
        
        # Calculate the Kalman Gain
        k1 = (self.C @ self.P) @ self.C.transpose() + self.V_noise
        K_f = (self.P @ self.C.transpose()) @ linalg.inv(k1)
        
        self.collectMeasurements()
        
        # Calculate Probable State & update X
        self.X = self.X_hat + K_f @ (self.Y_sensor - self.K_y()*self.Y_hat)
        
        # A posteriori probability
        self.P = (self.I - (K_f @ self.C) ) @ self.P
        
        # update F
        self.updateF()       
        
        return
    
    
    
    def saveFullData(self):
        '''Records and Saves VICON, Sensor, and Waypoint Data'''
        
        # Poll the Arduino to check & output the sensor data
        self.ser.write('1')
        
    	# probably don't need new variables?
        # give the Arduino a delay to send all of the data
        self.vX = self.pos[0][0]
        self.vY = self.pos[0][1]
        self.vTheta = self.pos[1][2] 
        self.vThetaDot = self.yaw_dot
        self.vSpeed = self.velocity_current 
        self.vTime = rospy.get_time()
        
        # Waypoint Data
        self.desiredX = self.position_desired[0]; self.desiredY = self.position_desired[1]
        self.desiredVhead = self.velocity_desired; self.desiredVlat = 0.0; 
        self.desiredTheta = yaw_next; self.desiredTheta_dot = yaw_traj_rate
        self.wTime = rospy.get_time()
        
        # Update state vector variables to see if needed to add to *.txt file
        # self.ViconState = [ self.vX, self.vY, self.vTheta, self.vThetaDot, self.vSpeed ]
        self.ViconState[0] = self.vX; self.ViconState[1] = self.vY; self.ViconState[2] = self.vTheta
        self.ViconState[3] = self.vThetaDot; self.ViconState[4] = self.vSpeed
        
        # self.wayPoint = [ self.desiredX, self.desiredY, self.desiredVhead, self.desiredVlat, ...
        #                   self.desiredTheta, self.desiredTheta_dot ]
        self.wayPoint[0] = self.desiredX; self.wayPoint[1] = self.desiredY; self.wayPoint[2] = self.desiredVhead
        self.wayPoint[3] = self.desiredVlat; self.wayPoint[4] = self.desiredTheta
        self.wayPoint[5] = self.desiredTheta_dot
        
        # Recieve the data from the Arduino
        while self.sensorString == '':
            self.sensorString = self.ser.readline()
            print('Trying to record data from Arduino')
        while self.sensorString[-1] != '\n':
            self.sensorString = self.sensorString + self.ser.readline()
        
        print(self.sensorString)
        self.sensorData = self.sensorString.split(',')
        print(self.sensorData)
        self.sVelo = float(self.sensorData[0]); self.sAccel0 = float(self.sensorData[1])
        self.sAccel1 = float(self.sensorData[2]); self.sAccel2 = float(self.sensorData[3])
        self.sMag0 = float(self.sensorData[4]); self.sMag1 = float(self.sensorData[5])
        self.sMag2 = float(self.sensorData[6]); self.Gyro0 = float(self.sensorData[7])
        self.sGyro1 = float(self.sensorData[8]); self.sGyro2 = float(self.sensorData[9])
        self.batteryVoltage = float(self.sensorData[10]); self.sTime = rospy.get_time()
        
        self.sensorString = ''
        
        # self.sensorState = [ self.sVelo, self.sAccel0, self.sAccel1, self.sAccel2, self.sMag0, ...
        #                      self.sMag1, self.sMag2, self.sGyro0, self.sGyro1, self.sGyro2 ]
        self.sensorState[0] = self.sVelo; self.sensorState[1] = self.sAccel0; self.sensorState[2] = self.sAccel1
        self.sensorState[3] = self.sAccel2; self.sensorState[4] = self.sMag0; self.sensorState[5] = self.sMag1
        self.sensorState[6] = self.sMag2; self.sensorState[7] = self.sGyro0; self.sensorState[8] = self.sGyro1
        self.sensorState[9] = self.sGyro2
		
        global file1, file2, file3
        
        saveData(file1, self.vTime, self.ViconState)
        saveData(file2, self.sTime, self.sensorState)
        print('Sensor State Vector:', self.sensorState)
        saveData(file3, self.wTime, self.wayPoint)
        
        return
    
    
    
    def updateF(self):
        '''Update the F Jacobian Matrix with the data from X'''
        self.F[0][2] = np.cos(self.X[4]); self.F[0][3] = -np.sin(self.X[4])
        self.F[0][4] = -self.X[2]*np.sin(self.X[4]) - self.X[3]*np.cos(self.X[4])
        self.F[1][2] = np.sin(self.X[4]); self.F[1][3] = np.cos(self.X[4])
        self.F[1][4] = self.X[2]*np.cos(self.X[4]) - self.X[3]*np.sin(self.X[4])
        self.F[2][3] = self.X[5]; self.F[2][5] = self.X[2]
        self.F[3][2] = self.X[5]; self.F[3][5] = self.X[3]
        
        return
        
        
        
    def collectMeasurements(self):
        ''' Collect the data from the sensors, run conversions, and compile into Y 
        (assumes sensors were paged already to save computational time) '''
        
        # Collect data from sensors
        while self.sensorString[-1] != '\n':
            print('Trying to record data')
            self.sensorString = self.sensorString + self.ser.readline()
            
        self.sensorData = self.sensorString.split(',')
        
        # TODO: Take out unnecessary variables
        self.sVelo = float(self.sensorData[0]); self.sAccel0 = float(self.sensorData[1])
        self.sAccel1 = float(self.sensorData[2]); self.sAccel2 = float(self.sensorData[3])
        self.sMag0 = float(self.sensorData[4]); self.sMag1 = float(self.sensorData[5])
        self.sMag2 = float(self.sensorData[6]); self.Gyro0 = float(self.sensorData[7])
        self.sGyro1 = float(self.sensorData[8]); self.sGyro2 = float(self.sensorData[9])
        
        
        # Convert the data to the right units
        self.sAccel0 *= 9.81/16384; self.sAccel1 *= 9.81/16384; self.sAccel2 *= 9.81/16384
        self.sMag0 *= 1; self.sMag1 *= 1; self.sMag2 *= 1    # TODO: Update these multipliers/conversions
        self.sGyro0 *= (np.pi/2)/10285; self.sGyro1 *= (np.pi/2)/10285; self.sGyro2 *= (np.pi/2)/10285
        
        
        # Compile into Y
        # TODO: Update these once the conversion variables are finalized
        self.Y[0] = self.sVelo; self.Y[1] = self.Y[1]
        self.Y[2] = self.Y[2]; self.Y[3] = self.Y[3]
        
        return
    
    
    def K_y(self):
        '''Unfinished: scales the predicted Y value based off how many loops have occurred
        since the last update from vicon (should be 0 < K_y <= 1)'''
        # TODO: Finish this with a nice function (maybe 1/x or something)
        return 1
            
        
            
        

########## SOCKET TO READ MAINFRAME DATA #########

def ReadMainframe():
    global lf
    global sock
    global origin
    origin = ('',0)
    t_prev = rospy.get_time()

	#create an IPV4 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

	#connection information
    info = ("", 2525)
	#bind the socket address
    sock.bind(info)
	#run loop
    run = True
    while run and not rospy.is_shutdown():
        dataOrig, origin = sock.recvfrom(1024) #up to 1024 bytes
	#	print "Data Orig: " + dataOrig
        if dataOrig == "END":
            print( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" )
            print( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" )
            print( "~~~~~~~~~~~~~~ENDING~~~~~~~~~~~~~~~~~" )
            print( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" )
            print( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" )
            lf.mainframe_mutex.acquire()
            lf.dataString = "END"
            lf.init = False
            lf.reset()
            lf.RGB = "100"
            lf.mainframe_mutex.release()
        else:	
            tokens = dataOrig.split("|")
            data = tokens[2]
            r0 = tokens[0]
            bias = tokens[1]
            if not lf.init:
                lf.RGB = "001"
                lf.t0 = rospy.get_time()
                lf.calib = float(bias)
                lf.r += float(r0)
                lf.init = True
                lf.time_old=0
                lf.time=0
                print( "steering bias:" + bias + ", initial distance: " + r0)
            else:
                lf.mainframe_mutex.acquire()
                lf.dataString = data
                lf.RGB = "010"
                lf.mainframe_mutex.release()
	

#		print "DT between mainframe msgs:\t", (rospy.get_time() - t_prev)
        t_prev = rospy.get_time()

	#clean up our socket
    sock.close() #todo - we need to restart this for the next experiment

    ########### ROS SERVICES ###########


def StateMessage():
	global lf
	global sock
	global origin

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if origin == ('',0):
			continue

		msg = ""
		#populate the StateService response data
		msg += str(rospy.get_time() - lf.t0) + ":"
		msg += str(lf.hostname) + ":"
		msg += str(lf.pos[0][0]) + ":" + str(lf.pos[0][1])  + ":"
		msg += str(lf.position_desired[0]) + ":"
		msg += str(lf.position_desired[1]) + ":"
		msg += str(lf.velocity_current) + ":"
		msg += str(lf.velocity_desired) + ":"
		msg += str(lf.soc) + ":"
		msg += str(lf.r) + ":"
		msg += str(lf.segment_id) + ":"

		sock.sendto(msg, origin)
		rate.sleep()

	sock.close()
    
def saveData(file_obj, time, stateVec):
    # global file_obj
    saveString = str(time)
    for ele in stateVec:
        saveString += '\t' + str(ele)        # use tabs so I can use np.genfromtxt() to analyze 
    saveString += '\n'; # print(saveString)
    file_obj.write(saveString); file_obj.flush()
    saveString = '\0'         # just for scoping stuff




if __name__ == '__main__':

#	GPIO.output(18,0)
#	GPIO.output(16,0)
#	GPIO.output(15,1)
	global lf
	lf = line_follower()

	comms = Thread(target = ReadMainframe)
	comms.start()

	th2 = Thread(target = StateMessage)
	th2.start()

	while not rospy.is_shutdown():
		lf.update()

	print("ROS Shutdown")

	GPIO.output(18,0)
	GPIO.output(16,0)
	GPIO.output(15,0)
	GPIO.cleanup()
#	lf.shutdown()



# Fin
