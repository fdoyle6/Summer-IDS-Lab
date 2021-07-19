import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

# Since we're using a weird way to calculate the variance, need to define a new variance function
def getVariance(x, mu):
    ''' 
    x is the measurement, mu is the mean/'correct' value
    (x & mu are assumed to be the same length)
    returns total variance
    '''
    if (len(x) != len(mu)):
        print("*** NOTE: VARIANCE CALCULATED WITH DIFFERENT LENGTH VECTORS ***")
        print("Len of x:", len(x), "; len of mu:", len(mu))
    
    return ( np.sum( (x-mu)**2 )/(len(mu) - 1) )

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

# Waypoint (Desired) Data
waypointT = waypointData[:,0] # NOTE: THIS IS JUST THE TIME THE POINT WAS EVALUATED
waypointX = waypointData[:,1]
waypointY = waypointData[:,2]
waypointVhead = waypointData[:,3]
waypointVlat = waypointData[:,4]
waypointTheta = waypointData[:,5]
waypointTheta_dot = waypointData[:,6]

# Sensor data unit conversion factors (time doesn't get scaled)
kVHead = 1.0
kAccel = 1.0
kMag = 1.0
kGyro = 1.0

# Update sensor data to the right units
sensorVHead *= kVHead
sensorAxs *= kAccel
sensorAys *= kAccel
sensorAzs *= kAccel
sensorMxs *= kMag
sensorMys *= kMag
sensorMzs *= kMag
sensorGxs *= kGyro
sensorGys *= kGyro
sensorGzs *= kGyro

''' The important X is X = [ x, y, v^H, v^L, theta, theta_dot ] and we are not paying attention to the measurements of x & y '''

# Calculate the x and y velocities from VICON
viconVx = np.zeros_like(viconT[:-1])
viconVy = np.zeros_like(viconT[:-1])

viconDeltaX = viconX[1:] - viconX[:-1]
viconDeltaY = viconY[1:] - viconY[:-1]
viconDeltaT = viconT[1:] - viconT[:-1]

for i in range(len(viconVx)):
    viconVx[i] = viconDeltaX[i]/viconDeltaT[i]
    viconVy[i] = viconDeltaY[i]/viconDeltaT[i]
  
# Calculate heading velocity
viconHeadingVector = np.array([np.cos(viconTheta), np.sin(viconTheta)])
viconVeloVector = np.array([viconVx, viconVy])
viconVHead = np.dot(viconHeadingVector[:-1], viconVeloVector)
  
# For a sanity check plot against the recorded speed graph
plt.plot(viconT[:-1], viconVx, label = 'V_x')
plt.plot(viconT[:-1], viconVy, label = 'V_y')
plt.plot(viconT, viconSpeed, label = 'Recorded Speed')
plt.plot(viconT[:-1], np.sqrt(viconVx**2 + viconVy**2), label = 'Calculated Speed')
plt.xlabel('Time'); plt.ylabel('Velocity/Speed'); plt.title('Figure 1: VICON Speed/Velo Comparison')
plt.legend(); plt.figure()

# initialize the sensor acceleration velocity vectors
sensorVxs = np.zeros_like(sensorT[:-1])
sensorVys = np.zeros_like(sensorT[:-1])
sensorVzs = np.zeros_like(sensorT[:-1])

# integrate the acceleration data cumulatively to find the velocity in each direction
sensorVxs = sp.integrate.cumtrapz(sensorAxs)
sensorVys = sp.integrate.cumtrapz(sensorAys)
sensorVzs = sp.integrate.cumtrapz(sensorAzs) #should (hopefully) stay around zero and be ignored eventually

# plot to compare velo's
plt.plot(viconT[:-1], viconVx, label = 'VICON V_x')
plt.plot(viconT[:-1], viconVy, label = 'VICON V_y')
plt.plot(sensorT[:-1], sensorVxs, label = 'Sensor V_x')
plt.plot(sensorT[:-1], sensorVys, label = 'Sensor V_y')
plt.plot(sensorT[:-1], sensorVzs, label = 'Sensor V_z')
plt.xlabel('Time'); plt.ylabel('Velocity'); plt.title('Figure 2: VICON vs Sensor Velo')
plt.legend(); plt.figure()

# plot to compare speeds
plt.plot(viconT[:-1], viconSpeed, label = 'VICON Speed')
plt.plot(viconT[:-1], np.sqrt(viconVx**2 + viconVy**2), label = 'VICON Calculated Speed')
plt.plot(viconT[:-1], viconVHead, label = 'VICON Heading Velocity')
plt.plot(sensorT[:-1], np.sqrt(sensorVxs**2 + sensorVys**2 + sensorVzs**2), label = 'Sensor Calculated Speed')
plt.plot(sensorT, sensorVHead, label = 'Encoder Speed')
plt.xlabel('Time'); plt.ylabel('Speed'); plt.title('Figure 3: VICON vs Sensor Speed & Heading Velo')
plt.legend(); plt.figure()

# TODO - FIGURE OUT IF THE ACCELERATIONS OUTPUT INTRINSICALLY OR EXTRINSICALLY

# Initialize variance variables
varVHead = 0.0
varVLat = 0.0
varTheta = 0.0
varTheta_dot = 0.0

# calculate variances




# Fin