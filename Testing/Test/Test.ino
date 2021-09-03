
#include "Shield2AMotor.h"  // Used to define the motor object
#include <Encoder.h>  // Used to define the encoders connected to the motors
#include <TimerOne.h> // Used for the timer interrupt 
#include <MPU6050_tockn.h>  // Used for the gyroscope MPU6050
#include <Wire.h> //  Used for the gyroscope MPU6050
#include <Servo.h>  // Used to control the gripper

#define PI 3.1415926535897932384626433832795  // define the math constant pi

int bluePower = 8;
const int TICKS_PER_CM_right = 281;  // the number of encoder counts per cm. obtained from phase A code
const int TICKS_PER_CM_left = 281;  // the number of encoder counts per cm. obtained from phase A code

const byte rightChannelA = 19;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte rightChannelB = 18;  // ... this is for the motor on the right.

const byte leftChannelA = 3;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte leftChannelB = 2;  // ... this is for the motor on the left.

Encoder leftEnc(leftChannelA, leftChannelB);  // using the encoder library, the left encoder is defined with it's pins.
Encoder rightEnc(rightChannelA, rightChannelB); // using the encoder library, the right encoder is defined with it's pins.

Shield2AMotor motors(SIGNED_MAGNITUDE); // used to control both motors 

//Servo myservo;  //An object of the servo library to control the position of the servo motor
//int pos = 0;  //Used to store the position of the gripper

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

const double KP=6;  // The proportional gain of the PID controller. the output should oscillate about the setpoint
const double KD=0;  // The derivative gain of the PID controller
const double KI=2;  // The integral gain of the PID controller
const double dt = 0.5; // let the timer be 500ms for the interrupt 

volatile double theta = 0.0;  // stores the orientation of the robot
volatile double headingError = 0.0; // stores the difference between desired orientation, theta, and current orientation
volatile double hyp = 0.0;  // stores the distance from the robot to the target position
volatile double gripperLength = 0.0;  // assumed the gripper is 4.0cm long
volatile double currentAngle = 0.0; // stores the angle received from the MPU6050;
volatile double angularVelocity = 0.0;  //  the angular velocity for turning functions
volatile double forwardSpeed = 0.0; // the speed to move the robot

//testing parameters for travelDistance() and desiredOrientation()
volatile double robot_x = 0.0;  // x coordinate of the robot
volatile double robot_y = 0.0;  // y coordinate of the robot
volatile double block_x = 10.0; // x coordinate of the block
volatile double block_y = 10.0; // y coordinate of the block

//openCV parameters
volatile double a; // from openCV: camera x coordinate of the robot
volatile double b; // from openCV: camera y coordinate of the robot
volatile double x; // from openCV: camera x coordinate of the yellow block
volatile double y; // from openCV: camera y coordinate of the yellow block
volatile double roboX; // from openCV: actual x coordinate of the robot
volatile double roboY; // from openCV: actual y coordinate of the robot

volatile double delta_left;
volatile double delta_right;
volatile double delta_orientation;
volatile double delta_distance;
volatile double orientation;
#define WHEELBASE 13.8  // in cm

const double pi_deg = 180.0;
volatile double tempAngle = 0.0;

volatile double error = 0.0;

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(dt*1000000); // The processor executes timerInterrupt every 50ms
  
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // the gyro gets its offsets in the first 3 seconds of the robot getting powered

  pinMode(bluePower, OUTPUT);
  digitalWrite(bluePower, HIGH);
}

void loop() {

if(Serial.available() > 0){
 char input = Serial.read(); //reads the information received on the serial monitor from python
  
  switch (input) {      //uses input for the switch case statement
    case ('x'):     //indicates the start of the X coordinate from python
      x=Serial.parseFloat();  //assigns the first float value to x before the next character.
      break;
    case ('y'):     //indicates the start of the Y coordinates read from python
    y = Serial.parseFloat();  //passes the next float value to y
    break;
     case ('a'):     //indicates the start of the a coordinates read from python
    a = Serial.parseFloat();  //passes the next float value to a
    break;

     case ('b'):     //indicates the start of the b coordinates read from python
    b = Serial.parseFloat();  //passes the next float value to b
    break;
    
    roboX = 0.0491*a + 59.818;
    roboY = -0.1928*b + 130.8; 


//  mpu6050.update();
//  tempAngle = mpu6050.getAngleZ();
//  if(tempAngle > pi_deg){  
//    currentAngle = tempAngle - 2*pi_deg;  
//  }
//  else if(tempAngle < -pi_deg){
//    currentAngle = tempAngle + 2*pi_deg;
//  }
//  else{
//      currentAngle = tempAngle;
//  }
//  Serial.print("right speed: "); Serial.print(rightSetpoint);  
//  Serial.print("\tleft speed: "); Serial.println(leftSetpoint);
//  Serial.print("current Angle: "); Serial.print(currentAngle);
//  Serial.print("\tdesired orientation: "); Serial.print(theta);   
//  Serial.print("\theadingError: "); Serial.println(headingError);

  forwardSpeed = 15.0;
  Serial.print("right speed: "); Serial.print(rightSetpoint);  
  Serial.print("\tleft speed: "); Serial.println(leftSetpoint);
}
}
}

void timerInterrupt(){ 
  theta = desiredOrientation(block_x, block_y, robot_x, robot_y); // calculate the angle that the robot should turn to face the block in a straight line
  hyp = travelDistance(block_x, block_y, robot_x, robot_y);
  adjustOrientation();  // adjust the robot's orientation and move
  leftSetpoint = forwardSpeed - angularVelocity;
  rightSetpoint = forwardSpeed + angularVelocity;
  speedControl();
}

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

  motors.control(-rightPWM, -leftPWM);
}

float velToPWMRight(int setpoint){
  if(setpoint > 0){
    float PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.00003 + b*0.0027 - c*0.1272 + setpoint*2.0841 + 4.9628;
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
    PWM = a*0.00003 + b*0.0027 - c*0.1272 + setpoint*2.0841 + 4.9628;
    return -(PWM);
  }
}

float velToPWMLeft(int setpoint){
  if(setpoint > 0){
    float PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0001 - b*0.0061 + c*0.0649 + setpoint*1.0249 + 5.0903;
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
    PWM = a*0.0001 - b*0.0061 + c*0.0649 + setpoint*1.0249 + 5.0903;
    return -(PWM);
  }
}

void adjustOrientation(){
  error = orientationError();
  if(abs(error) > 2.0){ // define a tolerance level so the robot isn't turning too much
    // if the error is greater than the tolerance level
    if(error > 0){
      angularVelocity = 10.0;
    }
    else{
      angularVelocity = -10.0;
    }
  }
  else{ // if the error is within the tolerance level
    if(hyp < 9){  // check if the robot is within 9cm of the block
    forwardSpeed = 0.0;  // stop the robot if it is very close to the block 
    //gripObject(); // open and close the grippers to hold the block
    }
    else{
      forwardSpeed = 25.0;  // move forward until the robot is very close to the block
      angularVelocity = 0.0;  // move straight without turning
    }
  }
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

double orientationError(){
  headingError = currentAngle - theta;  // calculate the difference between the angle the robot is facing and the angle it should be at to face the block in a straight line
  if(headingError > pi_deg){  // find the shortest angle the robot should turn 
    headingError = headingError - 2*pi_deg;  
  }
  if(headingError < -pi_deg){
    headingError = headingError + 2*pi_deg;
  }
  return headingError;
}

//double pixToCoordX(double a){
//  double x = 0.0491*a + 59.818;
//  return x;
//}
//
//double pixToCoordY(double b){
//  double y = -0.1928*b + 130.8;
//  return y; 
//}
