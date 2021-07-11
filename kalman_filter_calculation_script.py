# the script for the Extended Kalman Filter Model calculations (assumed to be inside a loop)

# in import statements
import numpy as np 
from scipy import linalg as l_alg

# Assume oldX is the known state of the system and we're trying
# to calculate X (X[i+1]) from oldX (X[i])
# X = np.array([x, y, vh, vl, theta, theta_dot]) # really will be 0's but here for readability
oldX = np.array([oldX1, oldX2, oldX3, oldX4, oldX5, oldX6]) # filled with the previous time-step's X
X_hat = np.zeros_like(oldX)

# use X_dot[i] = F[i]*X[i] + B*U[i]; X[i+1] = X_dot[i]*dt + X[i] (only need one X_dot)
X_dot = np.zeros_like(oldX)
dt = 1/1000 # sampling rate or clock frequency

# Control input - only one U vector because it gets recalculated independently later
# and the previous U does not need to be saved
U = np.array([ah, al, omega, omega_dot])

# System measurement output - only one Y for same reason as U
# Y = [vh, vl, theta, theta_dot]
Y_hat = np.array([0.0, 0.0, 0.0, 0.0])
Y = np.zeros_like(Y_hat)

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
X_hat = X_dot*dt + oldX

# Output Matrix (assumes Y[i] = C X[i] w/ no pass-though from U) 
''' is the Y = C X + 0 U assumption valid with U[2] = omega?? '''
C = np.diag(np.ones_like[X_hat])
C[0][0] = 0; C[1][1] = 0;

# Calculate Y
Y_hat = np.matmul(C, X_hat)

# Probability Calculations
# Variance Matrices (disturbance & noise, respectively)
V_dist = np.diag([ d_x, d_y, d_vh, d_vl, d_theta, d_theta_dot ])
V_noise = np.diag([ s_x, s_y, s_vh, s_vl, s_theta, s_theta_dot ])

# Kalman gain -> should either be 6x4 or 4x6 - which one is it?
K_f = np.zeros((6, 4), dtype = float)

# Solve Riccati equation
'''Is this technically a continuous or discrete system to calc the Kalman gain?'''
# What are the dimenstions of the solution of a Riccatti Equ?
Ric_Soln = np.zeros_like(X_hat)
Ric_Soln = l_alg.solve_continuous_are(F.transpose, C.transpose, V_dist, V_noise)

# Calculate Kalman gain
K_f = np.matmul(np.matmul(Ric_Soln, C.transpose), V_noise)

# Update X from Kalman Gain
'''Need to finish calculation steps'''




