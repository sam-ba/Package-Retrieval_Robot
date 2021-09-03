// This code takes in arbitrary coordinates of itself and a LEGO block, rotates to be in a straight line, calculates the distance to the block and moves forward until it is 
// close to the block. at the block a gripper is deployed to grab the block. 

#include "Shield2AMotor.h"  // Used to define the motor object
#include <Encoder.h>  // Used to define the encoders connected to the motors
#include <TimerOne.h> // Used for the timer interrupt 
#include <MPU6050_tockn.h>  // Used for the gyroscope MPU6050
#include <Wire.h> //  Used for the gyroscope MPU6050
#include <Servo.h>  // Used to control the gripper

#define PI 3.1415926535897932384626433832795  // define the math constant pi

const int TICKS_PER_CM_right = 281;  // the number of encoder counts per cm. obtained from phase A code
const int TICKS_PER_CM_left = 281;  // the number of encoder counts per cm. obtained from phase A code

const byte rightChannelA = 19;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte rightChannelB = 18;  // ... this is for the motor on the right.

const byte leftChannelA = 3;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte leftChannelB = 2;  // ... this is for the motor on the left.

Encoder leftEnc(leftChannelA, leftChannelB);  // using the encoder library, the left encoder is defined with it's pins.
Encoder rightEnc(rightChannelA, rightChannelB); // using the encoder library, the right encoder is defined with it's pins.

Shield2AMotor motors(SIGNED_MAGNITUDE); // used to control both motors 

Servo myservo;  //An object of the servo library to control the position of the servo motor
int pos = 0;  //Used to store the position of the gripper

MPU6050 mpu6050(Wire);  // an object of the MPU6050 library used to control the gyroscope

// Variables used to obtain the motor model for each motor
volatile long leftEncoder  = 0; // these variables are used in the timerInterrupt function to obtain the speed of the left motor in cm/s
volatile long prevLeftEncoder = 0;
volatile long changeLeftEncoder = 0;
volatile double changeLeftCentimeters = 0;
volatile double leftSpeed = 0;

volatile long rightEncoder  = 0;  // these variables are used in the timerInterrupt function to obtain the speed of the right motor in cm/s
volatile long prevRightEncoder = 0;
volatile long changeRightEncoder = 0;
volatile double changeRightCentimeters = 0;
volatile double rightSpeed = 0;

// For the PID control
volatile double leftError;
volatile double leftSetpoint;
volatile double changeInErrorLeft;
volatile double prevErrorLeft;
volatile double leftErrorSum;
volatile float leftPID;
int leftPWM;

volatile double rightError;
volatile double rightSetpoint;
volatile double changeInErrorRight;
volatile double prevErrorRight;
volatile double rightErrorSum;
volatile float rightPID;
int rightPWM;

const double KP=27;  // The proportional gain of the PID controller. the output should oscillate about the setpoint
const double KD=0;  // The derivative gain of the PID controller
const double KI=15;  // The integral gain of the PID controller
const double dt = 0.05; // let the timer be 500ms for the interrupt 

volatile double theta = 0.0;  // stores the orientation of the robot
volatile double headingError = 0.0; // stores the difference between desired orientation, theta, and current orientation
volatile double hyp = 0.0;  // stores the distance from the robot to the target position
volatile double gripperLength = 4.0;  // assumed the gripper is 4.0cm long
volatile double currentAngle = 0.0; // stores the angle received from the MPU6050;
volatile double angular_velocity = 2.0;  //  the angular velocity for turning functions

volatile double robot_x = 0.0;  // x coordinate of the robot
volatile double robot_y = 0.0;  // y coordinate of the robot
volatile double block_x = 10.0; // x coordinate of the block
volatile double block_y = 10.0; // y coordinate of the block

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(dt*1000000); // The processor executes timerInterrupt every 50ms
  
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");
 
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // the gyro gets its offsets in the first 3 seconds of the robot getting powered
  
  myservo.attach(9);  // the signal wire of the servo is attached to digital pin 9
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  currentAngle = abs(mpu6050.getAngleZ());
  Serial.print(rightSetpoint);  Serial.print("\t"); Serial.print(leftSetpoint); Serial.print("\t"); Serial.println(currentAngle);
}

void timerInterrupt(){   
  theta = desiredOrientation(block_x, block_y, robot_x, robot_y); // calculate the angle that the robot should turn to face the block in a straight line
  hyp = travelDistance(block_x, block_y, robot_x, robot_y); // calculate the straight line distance from the robot to the block
  //adjustOrientation();  // adjust the robot's orientation and move
  forward();
}

double desiredOrientation(double x_block, double y_block, double x_robot, double y_robot){
  double y = y_block - y_robot;
  double x = x_block - x_robot;
  double theta = atan2(y, x); // desired angle, heading
  theta = theta * (180/PI); // convert theta to degrees since the mpu also returns degrees
  return theta;
}

double travelDistance(double x_block, double y_block, double x_robot, double y_robot){
  double a = abs(x_block - x_robot);
  double b = pow(a, 2);
  double c = abs(y_block - y_robot);
  double d = pow(c, 2);
  double hyp = sqrt(b + d);
  return hyp;
}
/*
void adjustOrientation(){
  orientationError();
  if(abs(headingError) > 0.05){ // define a tolerance level so the robot isn't turning too much
    // if the error is greater than the tolerance level
    if(headingError < 0){
      left();
    }
    else{
      right();
    }
  }
  else{
    if(hyp < (gripperLength-0.5)){
    stopRobot();  // stop the robot if it is very close to the block 
    gripObject(); // open and close the grippers to hold the block
  }
    else{
      forward();  // move forward until the robot is very close to the block
    }
  }
}

float orientationError(){
  headingError = currentAngle - theta;  // calculate the difference between the angle the robot is facing and the angle it should be at to face the block in a straight line
  if(headingError > 2*PI){  // find the shortest angle the robot should turn 
    headingError = headingError - 2*PI;  
  }
  else {
    headingError = headingError + 2*PI;
  }
  return headingError;
}
*/
int speedControl(){
  // For the right motor
  rightEncoder = rightEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeRightEncoder = rightEncoder - prevRightEncoder;
  prevRightEncoder = rightEncoder;
  changeRightCentimeters = (float)changeRightEncoder/(float)TICKS_PER_CM_right; // convert encoder ticks to distance measurements
  rightSpeed = changeRightCentimeters/dt; // divide the distance measurements by 50ms and get speed
  //Update the PID terms
  rightError = rightSetpoint - rightSpeed;
  changeInErrorRight = rightError - prevErrorRight;
  prevErrorRight = rightError;
  rightErrorSum += rightError;

  rightPID = KP*rightError + KI*rightErrorSum*dt + changeInErrorRight*KD/dt;
  rightPWM = velToPWMRight(rightSetpoint) + rightPID;

  // For the left motor
  leftEncoder = leftEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeLeftEncoder = leftEncoder - prevLeftEncoder;
  prevLeftEncoder = leftEncoder;
  changeLeftCentimeters = (float)changeLeftEncoder/(float)TICKS_PER_CM_left;
  leftSpeed = changeLeftCentimeters/dt;
  //Update PID terms
  leftError = leftSetpoint - leftSpeed;
  changeInErrorLeft = leftError - prevErrorLeft;
  prevErrorLeft = leftError;
  leftErrorSum += leftError;

  leftPID = KP*leftError + KI*leftErrorSum*dt + changeInErrorLeft*KD/dt;
  leftPWM = velToPWMLeft(leftSetpoint) + leftPID;

  motors.control(rightPWM, leftPWM);
}

float velToPWMRight(int setpoint){
  if(setpoint > 0){
    float PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0438 - b*0.5417 + c*1.7452 + setpoint*4.9043 + 7.6844;
    return PWM;
  }
  if(setpoint == 0){
    return 0;
  }
  if(setpoint < 0){
    float PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0438 - b*0.5417 + c*1.7452 + setpoint*4.9043 + 7.6844;
    return -(PWM);
  }
}

float velToPWMLeft(int setpoint){
  if(setpoint > 0){
    float PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0075 + b*0.0504 - c*1.2217 + setpoint*9.6894 + 9.9903;
    return PWM;
  }
  if(setpoint == 0){
    return 0;
  }
  if(setpoint < 0){
    float PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0075 + b*0.0504 - c*1.2217 + setpoint*9.6894 + 9.9903;
    return -(PWM);
  }
}

void forward(){
  leftSetpoint = 5;
  rightSetpoint = 5;
  speedControl();
}

void stopRobot(){
  leftSetpoint = 0;
  rightSetpoint = 0;
  speedControl();
}

void left(){
  leftSetpoint = 0;
  rightSetpoint = 5;
  speedControl();
}

void right(){
  leftSetpoint = 5;
  rightSetpoint = 0;
  speedControl();
}

void gripObject(){
  // Open the gripper
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(15);                       // waits 15ms for the servo to reach the position but there can be no delay in an interrupt function
  }
  // Close the gripper
  for (pos = 180; pos >= 120; pos -= 1) { // goes from 180 degrees to 60 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(15);                       // waits 15ms for the servo to reach the position but there can be no delay in an interrupt function
  }
}
