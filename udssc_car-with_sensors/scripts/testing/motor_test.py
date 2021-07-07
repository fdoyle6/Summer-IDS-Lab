#!/usr/bin/env python

import time
import sys
import serial
import math


FUNCTIONALITY_TEST = "func"
SPEED_TEST = "speed"
MANUAL_COMMAND = "manual"

#ser = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=0)
ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout = 0)

def functionality_test():
	''' Linear and Sinusoidal Motor Testing '''

	# Extremity Testing
	# Steering
	ser.write("-45,0;")
	time.sleep(2)
	ser.write("45,0;")
	time.sleep(2)
	# Velocity
	ser.write("0,1;")
	time.sleep(2)
	ser.write("0,0;")
	time.sleep(2)

	# Sinusoidal Testing
	time_0 = time.time() # Reset t_0 b4 test so motor starts at rest; no snap
	duration = 2*math.pi # Duration in seconds
	eq = "45*math.sin(5*t)"

	# Steering Testing
	while time.time() < time_0 + duration:
		t = time.time() - time_0
		command = str(round(eval(eq),2)) + ",0;"
		command.encode(encoding="UTF-8",errors="strict")
		print round(t,2)
		old = 0
		ser.write(command)
		print(str(round(eval(eq),2)) + ",0;")
		old = round(t,2)
		time.sleep(0.05)
	ser.write("0,0;")
	
	# RX Read
	time_0 = time.time()
	print("RX Test:\n")
	while time.time() < time_0 + duration:
		soc = ser.readline()
		if soc:
			print soc

def speed_test(duration):
	''' Test max speed for the input duration.
	Intended for use with Vicon for testing acceleration
	and Arduino motor controller '''

	ser.write("0,1;")
	time.sleep(duration)
	ser.write("0,0;")

def manual_test(motor_cmd):
	''' Manually send a command to the car.
	Must be in w,v; format!!! '''

	valid = True
	test_string = motor_cmd

	# Check for ;
	if test_string[-1] != ";":
		valid = False
	else:
		test_string = test_string[:-1]

	# Check if two comma seperated values
	test_string = test_string.split(",")
	if len(test_string) != 2:
		valid = False

	# Check if values can be cast as float
	for i in test_string:
		try:
			float(i)
		except ValueError:
			valid = False

	if not valid:
		print "Command not in valid format.\nPlease follow this format:\n\
		'steering_angle,velocity;'"
		exit()
	else:
		ser.write(motor_cmd)

if __name__ == "__main__":

	try:
		mode = sys.argv[1]
	except IndexError:
		mode = "func"
	try:
		arg = sys.argv[2]
	except IndexError:
		arg = None

	##DEBUG:
	print "Mode: " + mode
	print "Arg: " + str(arg)

	if mode.lower() == FUNCTIONALITY_TEST:
		functionality_test()
	elif mode.lower() == SPEED_TEST and arg != None:
		try:
			float(arg)
		except ValueError:
			print "Error: Not a number.\nShutting down."
			exit()
		speed_test(float(arg))
	elif mode.lower() == SPEED_TEST and arg == None:
		speed_test(3.0)
	elif mode.lower() == MANUAL_COMMAND and arg != None:
		manual_test(arg)
	else:
		print "Invalid arguments. Please see readme."
