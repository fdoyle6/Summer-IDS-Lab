import numpy as np
from matplotlib import pyplot as plt
import scipy as sp

runNumber = ''
carNumber = ''

viconFileName = 'VICON_Data_Run_' + runNumber + '-Car_Number_' + carNumber + '.txt'
sensorFileName = 'Sensor_Data_Run_' + runNumber + '-Car_Number_' + carNumber + '.txt'

viconFile = open(viconFileName, 'r')
sensorFile = open(sensorFileName, 'r')
