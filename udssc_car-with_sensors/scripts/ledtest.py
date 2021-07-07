#!/usr/bin/env python

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,1)
GPIO.setup(16,GPIO.OUT)
GPIO.output(16,1)
GPIO.setup(15,GPIO.OUT)
GPIO.output(15,1)
try:
	while (True):
		request = raw_input("RGB-->")
		if (len(request)==3):
			GPIO.output(18,int(request[0]))
			GPIO.output(15,int(request[1]))
			GPIO.output(16,int(request[2]))
except KeyboardInterrupt:

	GPIO.cleanup()



