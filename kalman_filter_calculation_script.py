# the script for the Extended Kalman Filter Model calculations (assumed to be inside a loop)

# in import statements
import numpy as np 
from scipy import linalg as l_alg
from matplotlib import pyplot as plt

# just so the code will run
# Dummy state and control variables
oldX1 = 0; oldX2 = 1; oldX3 = 2; oldX4 = 3; oldX5 = 4; oldX6 = 5
ah = 0; al = 0; omega = 0; omega_dot = 0

# Dummy Uncertainties
d_x = 1; d_y = 1; d_vh = 1; d_vl = 1; d_theta = 1; d_theta_dot = 1
s_vh = 1; s_vl = 1; s_theta = 1; s_theta_dot = 1

# Dummy Sensor Readings
sensor1 = 2.1; sensor2 = 2.9; sensor3 = 4.0; sensor4 = 5.2

# Propogation of error coefficient
h = 1 # should update every loop through

# Assume oldX is the known state of the system and we're trying
# to calculate X (X[i+1]) from oldX (X[i])
# X = np.array([x, y, vh, vl, theta, theta_dot]) # really will be 0's but here for readability
oldX = np.array([ oldX1, oldX2, oldX3, oldX4, oldX5, oldX6 ]) # filled with the previous time-step's X
X_hat = np.zeros_like(oldX)

# use X_dot[i] = F[i]*X[i] + B*U[i]; X[i+1] = X_dot[i]*dt + X[i] (only need one X_dot)
X_dot = np.zeros_like(oldX)
dt = 25/1000 # sampling rate or clock frequency ***currently about 25 ms***

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

print(l_alg.eig(F))

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

# Output Matrix (Y[i] = C X[i] + 0 U[i])
C = np.zeros((4, 6))
C[0][2] = 1; C[1][3] = 1; C[2][4] = 1; C[3][5] = 1

# Calculate Y
Y_hat = np.matmul(C, X_hat)
Y[0] = sensor1; Y[1] = sensor2; Y[2] = sensor3; Y[3] = sensor4

# Probability Calculations
# Variance Matrices (disturbance & noise, respectively)
V_dist = np.diag([ d_x, d_y, d_vh, d_vl, d_theta, d_theta_dot ])
V_noise = np.diag([ s_vh, s_vl, s_theta, s_theta_dot ])

# Probability A Priori
P = np.zeros_like(X_hat)

# Calculate A Priori
P = np.matmul(np.matmul(F, P), F.transpose()) + V_dist

# Kalman gain -> should be 6x4
K_f = np.zeros((6, 4), dtype = float)

''' Omitted because reference was using dx_hat/dt = ...
# Solve Riccati equation assuming continuous system
# What are the dimenstions of the solution of a Riccatti Equ?
Ric_Soln = np.zeros_like(X_hat)
Ric_Soln = l_alg.solve_continuous_are(F.transpose(), C.transpose(), V_dist, V_noise) 

# Calculate Kalman gain
K_f = np.matmul(np.matmul(Ric_Soln, C.transpose), V_noise)
'''

# Calculate Kalman Gain
bigBoi = np.matmul(np.matmul(C, P), C.transpose()) + V_noise
K_f = np.matmul(np.matmul(P, C.transpose()), np.linalg.inv(bigBoi))

# Update the prediction & give the estimated state
X = X_hat + np.matmul(K_f, (Y - h*Y_hat))

# Calculate the final Probability for that location
P = np.matmul((1 - np.matmul(K_f, C)), P)

# Fin du calculation

# TODO - change over to the np.matrix class to help out with the notation



