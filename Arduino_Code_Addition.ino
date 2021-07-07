//Changes to /udssc_car-with_sensors/V4Car/V4Car.ino

//Line 6
#include <Zumo32U4IMU.h>

// ... Line 12
Zumo32U4IMU sensors;

//... Line 164
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
