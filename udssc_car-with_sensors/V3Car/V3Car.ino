
#include <Servo.h>                //Include servo library
#include <Encoder.h>              //Include encoder library
#define AIN1      5        //Setting MOTOR_IN1 to pin 5 on the arduino
#define AIN2      6        //Setting MOTOR_IN2 to pin 6 on the arduino
#define PWMA      11
#define STBY      4
#define encoder0PinA      2       // encoder A pin
#define encoder0PinB      3       // encoder B pin
//#define currentSensorPin

//
double lastTime = millis();       // sets lastTime to the current time when the program starts
String input;                     //intialize input variable

//Rear motor variables
volatile double count = 0;          // Rev counter
String inputVelocityString;         // motorInput string from ROS JOY
double  inputVelocity = 0.0;
double actualVelocity = 0;
double rotationRate = 0;
volatile double rotations = 0;

//Current Sensor Variables
int currentRaw = 0;
int currentCount = 0;


//servo variables
Servo myservo;                    // Create servo object to control a servo
int pos = 0;                      // Variable to store the servo position
String inputSteering;             // Steering string input from ROS JOY
double desiredSteering = 90.00;    //Initailizing steering to 0

//relevent constants
double wheelCir = 0.0996199;
int direction = 0;
double ts = 1000.0;
double cycleTime = 25.00;           //milliseconds (1000/second)
//controller gains
//double ku = 2000;
//double tu = 0.1;
double kp = 80;//.6*ku;
double ki = 24;//1.2*ku/tu;
double kd = 40;//3*ku*tu/40;
/*
  original : 80, 24, 40
  1 : 80, 1 , 0 - Slow build to stable 50
  2 : 80, 12, 0 - drastic overshoot, over-occilation
  3 : 80, 4, 0 - better draw to speed, but overshoot present
  4 : 80, 6, 0 -
*/

//motor read variables
int currentSensorPin = A3;
int socPin = A1;



//control variables;
double error = 0;
double prevError = 0;
double integral = 0;
double derivative = 0;
double pidTerm = 0;
int finalInput = 0;
double current = 0;

//takes note of rotations b/w cycleTimes
void indexRise() {
  count += 1;
  //Serial.println("Encoder working");
}



//match motor and servo to pins
void setup() {
  Serial.begin(115200);
  myservo.attach(9);                                           // attaches the servo on pin 9 to the servo object
  pinMode(AIN1, OUTPUT); https: //learn.sparkfun.com/tutorials/sik-experiment-guide-for-the-arduino-101genuino-101-board/experiment-4-driving-multiple-leds
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), indexRise, RISING);
  //write 0 to everything initially
  //analogWrite(PWMA, 0);
  myservo.write(90);
}


/*

   Take input v,w;
   INPUT FORMAT MUST BE string where 'w,v;'
   v is the car's velocity of range [+/- about 0.8 m/s] where
    v > 0 is car going forward
    v = 0 is brake
    v < 0 is in reverse
   w it the cars steering angle of range[-4,4] where
    w < 0 is a left turn
    w = 0 is a straight path
    w > 0 is a right turn
*/
void loop() {

  if (Serial.available() > 0) {
    inputSteering = Serial.readStringUntil(','); //
    desiredSteering = inputSteering.toDouble() + (90);          //Can accept values from -4-4... change turnscalar to 4
    inputVelocityString = Serial.readStringUntil(';');            //takes velocity value as string
    inputVelocity = inputVelocityString.toDouble();               //converts to double
    if (inputVelocity < 0) {                                      //checks the sign of the inputted velocity and saves it as direction then converts input velocity to postive if needed
      direction = -1;
      inputVelocity = abs(inputVelocity);
    }
    else if (inputVelocity > 0 ) {
      direction = 1;//
    } else {
      direction = 0;
      
    }
  }


  //set the velocity to the desired velocity

  if (direction > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(STBY, HIGH);
    //digitalWrite(PWMA, HIGH);

  }
  else if (direction < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(STBY, HIGH);
    //digitalWrite(PWMA, HIGH);

  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(STBY, LOW);
    error = 0;
    integral = 0;
    derivative = 0;
    count = 0;
    prevError = 0;
    pidTerm = 0;


    //digitalWrite(PWMA, LOW);
    //Serial.println("Break!");
  }

  //Update Velocity Reading
  if ((millis() - lastTime) > cycleTime) {
    rotations = count / 225.0;                                      //1 rotation = 225 counts from encoder
    //Serial.print("Counts: ");Serial.println(count);
    rotationRate = (rotations / (millis() - lastTime)) * (ts );   //counts/ses
    actualVelocity = rotationRate * wheelCir;                     //v = change in counts / (current time - timeSinceLast)*1000/ time between cycles
    lastTime = millis();
    count = 0;
    rotations = 0;
    // ***** SOC stuff ***** //
    float soc = analogRead(socPin) / 1023.0f * 5.03; //convert 1024 bits to 5V measurement
    soc = soc * 2; //we're reading half the battery voltage
    Serial.println(soc);
    //Serial.println("P: " + String(error) + " I: " + String(integral) + " D: " + String(derivative) + " Time: " + String(millis()/1000.0) + " pidTerm: " + String(pidTerm));
    //Serial.println("Actual Velocity: " + String(actualVelocity) + " Input Velocity: " + String(inputVelocity));
    currentRaw += analogRead(currentSensorPin);
    //currentCount +=1;

  }

  //Calcualte current
  //current = currentRaw / currentCount;

  //calcuate pid term
  error = abs(inputVelocity) - abs(actualVelocity)  ;             //actual velocity - desrired velocity, converted back to motor power terms
  integral = integral + (error * cycleTime / 1000.0);
  integral = constrain(integral, -90, 90);
  derivative = ((error - prevError) / (cycleTime / 1000.0));
  pidTerm = (kp * error) + (ki * integral) + (kd * derivative);
  prevError = error;



  //constrains the terms to make inout valid
  pidTerm = constrain(pidTerm, 0, 255 );
  desiredSteering = constrain(desiredSteering, 45, 135);                           //Needs to be adjusted depending on the servo..

  //Serial.println(pidTerm);
  //input into motor
  analogWrite(PWMA, pidTerm);
  //Serial.print("Velocity: ");
  //Serial.println( actualVelocity);
  //Serial.print("PIDTerm: ");
  //Serial.println( pidTerm);
  //Serial.println("P: " + String(error)); //+ " I: " + String(integral) + " D: " + String(derivative) + " Time: " + String(millis()/1000.0));
  myservo.write(desiredSteering);
  //Serial.println(currentRaw);


}
