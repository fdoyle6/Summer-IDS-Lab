#ifndef MOTORCOMMANDER_HPP
#define MOTORCOMMANDER_HPP

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

class MotorCommander {
		char serialPortFilename[];
	public:
		MotorCommander();
		~MotorCommander();
		void sendCommand(float velocity, float steering_angle);
};

#endif
