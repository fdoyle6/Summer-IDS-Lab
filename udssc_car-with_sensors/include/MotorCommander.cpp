#include "MotorCommander.hpp"

MotorCommander::MotorCommander() {
	std::cout << "MotorCommander created" << std::endl;
	arduino.open()
}

MotorCommander::~MotorCommander() {
	std::cout << "MotorCommander deleted" << std::endl;
	sendCommand(0.0f,0.0f);
	arduino.close();
}

void MotorCommander::sendCommand(float velocity, float steering_angle) {
	std::cout << "SLEEP FOR FIVE SECONDS" << std::endl;
	sleep(5);
	std::cout << "NOW TRYNA SEND COMMAND YO" << std::endl;
	std::string command;
	command = "0.5,0.0;";
	arduino << command;
}
