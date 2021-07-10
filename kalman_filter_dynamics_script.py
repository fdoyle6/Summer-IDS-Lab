# the script for the Extended Kalman Filter Model calculations (assumed to be inside a loop)

import numpy as np

# Assume oldX is the known state of the system and we're trying
# to calculate X (X[i+1]) from oldX (X[i])
# X = np.array([x, y, vh, vl, theta, theta_dot]) # really will be 0's but here for readability
oldX = np.array([oldX1, oldX2, oldX3, oldX4, oldX5, oldX6]) # filled with the previous time-step's X
X = np.zeros_like(oldX)

# use X_dot[i] = F[i]*X[i] + B*U[i]; X[i+1] = X_dot[i]*dt + X[i] (only need one X_dot)
X_dot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dt = 1/1000 # sampling rate or clock frequency

# Control input - only one U vector because it gets recalculated later
U = np.array([ah, al, omega, omega_dot])

# dynamical F Jacobian Matrix - uses bike model
F = np.array([ [0, 0, np.cos(oldX[4]), -np.sin(oldX[4]), -oldX[2]*np.sin(oldX[4])-oldX[3]*np.cos(oldX[4]), 0],
     [0, 0, np.sin(oldX[4]), np.cos(oldX[4]), oldX[2]*np.cos(oldX[4])-oldX[3]*np.sin(oldX[4]), 0],
     [0, 0, 0, oldX[5], 0, oldX[2]], 
     [0, 0, oldX[5], 0, 0, oldX[3]],
     [0, 0, 0, 0, 0, 1],
     [0, 0, 0, 0, 0, 0] ])

# control input matrix
B = np.array([ [0, 0, 0, 0],
     [0, 0, 0, 0],
     [1, 0, 0, 0],
     [0, 1, 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1] ])

# Dynamical Model Predictions
X_dot = np.matmul(F, oldX) + np.matmul(B, U)
X = X_dot*dt + oldX

