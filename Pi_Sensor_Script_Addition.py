# Designed to go in UD-IDS-LAB/udssc_car/scripts/path_controller.py
# This will probably need some tweaking before it goes into that code though


''' All sensor data comes through the serial port from the arduino
Structure: sensorData = array of doubles with length of 80 bytes
        sensorData[0:8] - heading velocity
        sensorData[8:32] - acceleration (vector)
                a1 = sensorData[8:16]; a2 = sensorData[16:24]; a3 = sensorData[24:32]
        sensorData[32:56] - orientation (vector)
                (each element 8 bytes long)
        sensorData[56:80] - angular velo (vector)
                (each element 8 bytes long) '''


# ** NOTE: HOW I'M DOING THIS MAY INTERFERE WITH SOME FUNCTIONS LIKE getSOC **

# line ~38 ----------------
# Data file names - NOTE: Change N & X each run
global file1_name, file2_name, f1, f2
file1_name = 'VICON_Data_Run_N-Car_Number_X.txt'
file2_name = 'Sensor_Data_Run_N-Car_Number_X.txt'

# line ~200 ---------------------
# Create files
global f1 = open(file1_name, 'w')         # file for VICON data
global f2 = open(file2_name, 'w')         # file for sensor data

# Add headers to data files
saveData(f1, 'Time', ['X', 'Y', 'Heading Angle', 'Angular Velocity', 'Speed'])
saveData(f2, 'Time', ['Heading Velocity', 'Acceleration 1', 'Acceleration 2', 'Acceleration 3', 'Compass 1', 'Compass 2', 'Compass 3', 'Gyro 1', 'Gyro 2', 'Gyro 3'])

# 2 components of acceleration, 2 compass components, and 2 gyro components are useless 
# for right now, but I'm keeping all of them so I know how the sensors work completely. 

''' ----- Running variables: should I put them in __init__ (~line 117) & change them to self.variable or take the already defined variables out? --------- '''
# sensor and vicon data variables
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
		self.sTime = rospy.get_time(); 

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

		# comparison vectors (time not included in state vector)
		self.ViconState = [ vX, vY, vTheta, vThetaDot, vSpeed ]
		self.sensorState = [ sVelo, sAccel0, sAccel1, sAccel2, sMag0, sMag1, sMag2, sGyro0, sGyro0, sGyro2 ]

		self.oldViconState =  [ o_vX, o_vY, o_vTheta, o_vThetaDot, o_vSpeed ]
		self.oldSensorState = [ o_sVelo, o_sAccel0, o_sAccel1, o_sAccel2, o_sMag0, o_sMag1, o_sMag2, o_sGyro0, o_sGyro1, o_sGyro2 ]

# In Update ~line 460 (used inside a loop) --------------------------------------------------
# probably don't need new variables?
		self.vX = self.pos[0][0]
		self.vY = self.pos[0][1]
		self.vTheta = self.pos[1][2] 
		self.vThetaDot = self.yaw_dot
		self.vSpeed = self.velocity_current 
		self.vTime = rospy.get_time()

		# I may need to export the data from the arduino as a string instead of an array of doubles
		self.sVelo = self.ser.read(8)
		self.sAccel0 = self.ser.read(8)
		self.sAccel1 = self.ser.read(8)
		self.sAccel2 = self.ser.read(8)
		self.sMag0 = self.ser.read(8) 
		self.sMag1 = self.ser.read(8)
		self.sMag2 = self.ser.read(8)
		self.sGyro0 = self.ser.read(8)
		self.sGyro1 = self.ser.read(8)
		self.sGyro2 = self.ser.read(8)
		self.sTime = rospy.get_time()

		self.ViconState = [ self.vX, self.vY, self.vTheta, self.vThetaDot, self.vSpeed ]
		self.sensorState = [ self.sVelo, self.sAccel0, self.sAccel1, self.sAccel2, self.sMag0, 
		self.sMag1, self.sMag2, self.sGyro0, self.sGyro1, self.sGyro2 ]
        
		if not (np.allclose(ViconState, oldViconState)):
        		saveData(f1, vTime, ViconState)
        		oldViconState = ViconState; o_vTime = vTime

		if not (np.allclose(sensorState, oldSensorState)):
        		saveData(f2, sTime, sensorState)
        		oldSensorState = sensorState; o_sTime = sTime
# in shutdown before exit() ~line 625 -----------------------------
f1.close(); f2.close()


# Before if __name__ == '__main__' ~line 805 -------------------------
def saveData(file_obj, time, stateVec):
        saveString = str(time)
        for ele in stateVec:
                saveString.append('\t' + str(ele))         #use tabs so I can use np.genfromtxt() to analyze 
        saveString.append('\n')
        file_obj.write(saveString); saveString = '\0'         # just for scoping stuff




# Fin.
