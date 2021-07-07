//Goes in UD-IDS-LAB/udssc_car/V4Car/V4Car.ino

// After line 6
#include <Zumo32U4IMU.h>

// ... line ~61
//sensor variables
double sensorData[80];


//... ~ line 160 (before PID control):
// Data Collection and Export ------------------------------------------------------------------------------------
read(); //read accel, mag, & gyro data; velo data already found


sensorData[0] = actualVelocity;
sensorData[8] = a; sensorData[32] = m; sensorData[56] = g;
Serial.write(sensorData, 80);
//may need to send sensor data as string if weird with being doubles

//if this doesn’t work follow this link for steps to finish examples
