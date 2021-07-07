#!/usr/bin/env python
import os
import rospy
import serial
import time

if __name__ == '__main__':

	bias = '0'
	print "hello! "
#	ser = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=0)
#	ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout = 0)
	ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout = 0)
	ser.write(bias+',0;')
	time.sleep(1.0)
	for i in range(9):
		print ser.readline()
		msg = bias+',0.2;'
		ser.write(msg)
		print msg
		time.sleep(1)
	ser.write(bias+',0;')
	time.sleep(1.0)
	ser.write('0,-1;')
	ser.write('0,0;')
