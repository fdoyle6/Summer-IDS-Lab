//Goes in UD-IDS-LAB/udssc_car/V4Car/V4Car.ino

// After line 6
#include <Zumo32U4IMU.h>

Zumo32U4IMU sensors;

//... ~ line 160 (before PID control):
// Data Collection and Export ------------------------------------------------------------------------------------
  sensors.read(); //read accel, mag, & gyro data; velo data already found 

  Serial.write(actualVelocity);
  Serial.write(sensors.a.x);
  Serial.write(sensors.a.y);
  Serial.write(sensors.a.z);
  Serial.write(sensors.m.x);
  Serial.write(sensors.m.y);
  Serial.write(sensors.m.z);
  Serial.write(sensors.g.x);
  Serial.write(sensors.g.y);
  Serial.write(sensors.g.z);
