import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

# create file names and open file data
runNumber = '0'
carNumber = '0'

viconFileName = 'VICON_Data_Run_' + runNumber + '-Car_Number_' + carNumber + '.txt'
sensorFileName = 'Sensor_Data_Run_' + runNumber + '-Car_Number_' + carNumber + '.txt'
waypointFileName = 'Waypoint_Data-Run_' + runNumber + '-Car_Number_' + carNumber +'.txt'

viconData = np.genfromtxt(viconFileName, skip_header=1)
sensorData = np.genfromtxt(sensorFileName, skip_header=1)
waypointData = np.genfromtxt(waypointFileName, skip_header=1)

# Take data and divide it into pieces
# Vicon data
viconT = viconData[:,0]
viconX = viconData[:,1]
viconY = viconData[:,2]
viconTheta = viconData[:,3]
viconTheta_dot = viconData[:,4]
viconSpeed = viconData[:,5]

# The x, y, & z axes are probably weirdly aligned with the vicon x & y so thats why there's a weird name here
# Sensor Data
sensorT = sensorData[:,0]
sensorVHead = sensorData[:,1]
sensorAxs = sensorData[:,2]
sensorAys = sensorData[:,3]
sensorAzs = sensorData[:,4]
sensorMxs = sensorData[:,5]
sensorMys = sensorData[:,6]
sensorMzs = sensorData[:,7]
sensorGxs = sensorData[:,8]
sensorGys = sensorData[:,9]
sensorGzs = sensorData[:,10]

# Waypoint Data
waypointT = waypointData[:,0]
waypointX = waypointData[:,1]
waypointY = waypointData[:,2]

''' The important X is X = [ x, y, v^H, v^L, theta, theta_dot ] and we are not paying attention to the measurements of x & y '''

# Calculate the x and y velocities from VICON
viconVx = np.zeros_like(viconT)
viconVy = np.zeros_like(viconT)

viconDeltaX = viconX[1:] - viconX[:-1]
viconDeltaY = viconY[1:] - viconY[:-1]
viconDeltaT = viconT[1:] - viconT[:-1]

for i in range(len(viconVx)):
  viconVx[i] = viconDeltaX[i]/viconDeltaT[i]
  viconVy[i] = viconDeltaY[i]/viconDeltaT[i]
  
# For a sanity check plot against the recorded speed graph
plt.plot(viconT[:-1], viconVx, label = 'V_x')
plt.plot(viconT[:-1], viconVy, label = 'V_y')
plt.plot(viconT, viconSpeed, label = 'Recorded Speed')
plt.plot(viconT[:-1], np.sqrt(viconVx**2 + viconVy**2), label = 'Calculated Speed')
plt.xlabel('Time'); plt.ylabel('Velocity/Speed'); plt.legend(); plt.figure()

sensorVxs = sp.integrate.cumtrapz(sensorAxs)
sensorVys = sp.integrate.cumtrapz(sensorAys)
sensorVzs = sp.integrate.cumtrapz(sensorAzs)

