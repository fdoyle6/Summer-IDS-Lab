/* USES MODIFIED SERVO LIBRARY -> USE TIMER3 INSTEAD OF TIMER1 ON ZUMO!  https://www.pololu.com/docs/0J63/3.11.1 Otherwise it breaks every thing!!! --Behdad*/

#include <Servo.h>                //Include servo library
#include <Encoder.h>              //Include encoder library
#include <Wire.h>
#include <Zumo32U4.h>       //Include zumo motors
#include <Zumo32U4IMU.h>

#define encoder0PinA      0      // encoder A pin
#define encoder0PinB      1       // encoder B pin

Zumo32U4Motors motors;
Zumo32U4IMU sensors;

double lastTime = millis();       // sets lastTime to the current time when the program starts
String input;                     //intialize input variable

//Rear motor variables
volatile double count = 0;          // Rev counter
String inputVelocityString;         // motorInput string from ROS JOY
double  inputVelocity = 0;
double actualVelocity = 0;
double rotationRate = 0;
volatile double rotations = 0;

//Battery variables
double voltageZ = 0;              // Voltage as read by Zumo
double motorVoltage = 5.9;       // The safe voltage limit for the motor

//servo variables
Servo myservo;                    // Create servo object to control a servo
int pos = 0;                      // Variable to store the servo position
String inputSteering;             // Steering string input from ROS JOY
double desiredSteering = 90.00;    //Initailizing steering to 0
double desiredSteeringRaw;  // this is the angle recieved from Ras.py between -45 , 45

//relevant constants
double wheelCir = 0.0996199;
int direction = 0;
double ts = 1000;
double cycleTime = 25.00;           //milliseconds (1000/second)

//controller gains
double kp = 225;
double ki= 125;
double kd = 0;

/*
 *1 : 1900, 300, 900 | Small rapid oscillations
 *2 : 1900, 45, 900  | Oscillates at lower speeds, doesn't reach high speeds
 *3 : 1900, 50, 900  | Changed i overflow, 30 -> 10
 *4 :
 */

//control variables;
double error = 0;
double prevError = 0;
double integral = 0;
double derivative = 0;
double pidTerm = 0;
int finalInput = 0;
double current = 0;

//takes note of rotations b/w cycleTimes
void indexRise(){
  count += 1;
  // Serial.println("Encoder working");
}

//match motor and servo to pins
void setup() {
  Serial.begin(115200);
  motors.flipRightMotor(true);
  myservo.attach(4,1000,2000); // attaches the servo on pin 4 to the servo object https://www.arduino.cc/en/Reference/ServoAttach https://servodatabase.com/servo/towerpro/sg90
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), indexRise, RISING);
  myservo.write(110);
  Wire.begin();
  sensors.init();
}


/*
 *rising time w/ significant overshoot
 * Take input v,w;https://files.pololu.com/arduino/package_pololu_index.json
 * INPUT FORMAT MUST BE string where 'w,v;'
 * v is the car's velocity of range [+/- about 0.8 m/s] where
    v > 0 is car going forward * direction
    v = 0 is brake  motors.setRightSpeed(pidTerm);
    v < 0 is in reverse
 * w it the cars steering angle of rang  e[-4,4] where
    w < 0 is a left turn
    w = 0 is a straight path
    w > 0 is a right turn
 */
void loop() {

  // *************** INTERPRET COMMAND ******************** //
  if(Serial.available() > 1) { //changed to 1 -> no input code needed for steering
    inputSteering = Serial.readStringUntil(','); //
    desiredSteeringRaw = inputSteering.toDouble();
    //desiredSteering = -inputSteering.toDouble() + (90);          //Can accept values from -4-4... change turnscalar to 4
    if (desiredSteeringRaw >0){
      //desiredSteering = ((45-desiredSteeringRaw)/45)*(110-90) + 90 ; //90 to servo corresponds to 45 (full left), 110 to servo corresponds to 0
      desiredSteering = ((90.00-15.00)/(0-45.00))*(desiredSteeringRaw-45.00)+15.00; // 15 to servo corresponds to 45 (full left), 90 to servo corresponds to 0
      
    }else { 
      //desiredSteering = (desiredSteeringRaw/45)*(110-160) + 110; //160 to servo corresponds to -45 (full right), 110 to servo corresponds to 0
      desiredSteering = ((180.00-90.00)/(-45.00-0))*(desiredSteeringRaw-0)+90.00; // 180 to servo corresponds to -45 (full right), 90 to servo corresponds to 0

      }
      
    inputVelocityString = Serial.readStringUntil(';');            //takes velocity value as string
    inputVelocity = inputVelocityString.toDouble();               //converts to double
    if(inputVelocity < 0){                                        //checks the sign of the inputted velocity and saves it as direction then converts input velocity to postive if needed
      direction = -1;
      inputVelocity = abs(inputVelocity);
      motors.flipRightMotor(false);
    }
    else if(inputVelocity > 0 ){
      direction = 1;
      motors.flipRightMotor(true);
    }else{
      direction = 0;
    }
  } 

  // *********** BATTERY VOLTAGE READING ************* //
  // This part reads the battery voltage through the Zumo
  // for use with the motor controller. This is NOT a reliable
  // reading to get the battery level, since the batteries we're
  // currently using have a non-linear discharge.
  // See: https://www.richtek.com/battery-management/img/battery-discharge.png
  // This voltage is scaled down to motorVoltage so the motor doesn't burn out
  
  voltageZ = readBatteryMillivolts()/1000;
  //Serial.println(readBatteryMillivolts());
  //Serial.println(voltageZ); //need to add this to the sent vector later

  //Serial.println("Desired Steering after serial:");
  //Serial.println(desiredSteering);

  // ***************** UPDATE ACTUAL VELOCITY ***************** //
  if ((millis() - lastTime) > cycleTime){
    rotations = count / 225;                                      //1 rotation = 225 counts from encoder
    // Serial.print("Counts: ");Serial.println(count);
    rotationRate = (rotations / (millis() - lastTime)) * (ts );   //counts/ses
    actualVelocity = rotationRate * wheelCir;                     //v = change in counts / (current time - timeSinceLast)*1000/ time between cycles
    lastTime = millis();
    count = 0;
    rotations = 0;

  }


  // Data Collection and Export ------------------------------------------------------------------------------------
  if (Serial.available() == 1) { //may need to change to 2
    sensors.read(); //read accel, mag, & gyro data; velo data already found 
    
    Serial.print(actualVelocity, 6); Serial.print(','); // keep 6 decimal places size = 4
    Serial.print(sensors.a.x); Serial.print(',');      //size = 2
    Serial.print(sensors.a.y); Serial.print(',');       //size = 2
    Serial.print(sensors.a.z); Serial.print(',');       //size = 2
    Serial.print(sensors.m.x); Serial.print(',');       //size = 2
    Serial.print(sensors.m.y); Serial.print(',');       //size = 2
    Serial.print(sensors.m.z); Serial.print(',');       //size = 2
    Serial.print(sensors.g.x); Serial.print(',');       //size = 2
    Serial.print(sensors.g.y); Serial.print(',');       //size = 2
    Serial.print(sensors.g.z); Serial.print(',');       //size = 2
    Serial.print(voltageZ); Serial.print(','); Serial.print(';'); //just for the split command and having a definite finish
  }


  // ****************** PID CONTROL ********************** //
  error = abs(inputVelocity) - abs(actualVelocity)  ;             //actual velocity - desrired velocity, converted back to motor power terms
  integral = integral + (error * cycleTime / 1000);
  integral = constrain(integral, -4, 4);
  derivative = ((error - prevError) / (cycleTime / 1000));
  pidTerm = (kp * error) + (ki * integral) + (kd * derivative);
  prevError = error;


  //constrains the terms to make inout valid
  pidTerm = constrain(pidTerm, 0, motorVoltage/voltageZ*400);                           // Constrain our pidTerm to the safe voltage.. 
  desiredSteering = constrain(desiredSteering,15,180);                             //This is just for safety of servo
  //delay(5000);
  //Serial.println(pidTerm);
  //input into motor
  //Serial.println("SPEED SET"); build tools
  //analogWrite(PWMA, pidTerm);
  //Serial.println("ACTUAL VELOCITY: ");
//  // PID TUNING //
//  Serial.print(actualVelocity);
//  Serial.print(" ");
//  Serial.print(inputVelocity);
//  Serial.print(" |");
//  Serial.print(" P:");
//  Serial.print(kp*error);
//  Serial.print(" I:");
//  Serial.print(ki*integral);
//  Serial.print(" D:");
//  Serial.println(kd*derivative);
  /////////////////
  //Serial.println("PID TERM: ");
  //Serial.println(pidTerm);
  // Serial.println("DESIRED VELOCITY");
  // Serial.println(inputVelocity);
  if (inputVelocityString[0] == '-' && inputVelocityString[1] == '0') {
    //Serial.println(inputVelocityString[0]);
    //integral = 0;
    motors.setRightSpeed(0);
  } else {
    motors.setRightSpeed(pidTerm);
  }
  // TEMP: Redefine limits for new steering mechanism
  // Serial.println(desiredSteering);
  myservo.write(desiredSteering);
  // Serial.println(currentRaw);


}
