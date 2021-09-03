#include "Shield2AMotor.h"
#include "CytronMotorDriver.h"
#include <Encoder.h>  // Used to define the encoders connected to the motors
#include <TimerOne.h> // Used for the timer interrupt 
#include <MPU6050_tockn.h>  // Used for the gyroscope MPU6050
#include <Wire.h> //  Used for the gyroscope MPU6050
#include <Servo.h>  // Used to control the gripper

const int TICKS_PER_CM_right = 281;  // the number of encoder counts per cm. obtained from phase A code
const int TICKS_PER_CM_left = 281;  // the number of encoder counts per cm. obtained from phase A code

const byte rightChannelA = 19;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte rightChannelB = 18;  // ... this is for the motor on the right.

const byte leftChannelA = 3;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte leftChannelB = 2;  // ... this is for the motor on the left.

Encoder leftEnc(leftChannelA, leftChannelB);  // using the encoder library, the left encoder is defined with it's pins.
Encoder rightEnc(rightChannelA, rightChannelB); // using the encoder library, the right encoder is defined with it's pins.

//Shield2AMotor motor1(SIGNED_MAGNITUDE); // the motor on the right is defined as an object of the Shield2AMotor library.
//Shield2AMotor motor2(SIGNED_MAGNITUDE); // the motor on the left is defined as an object of the Shield2AMotor library. 
Shield2AMotor motors(SIGNED_MAGNITUDE); // used to control both motors 
//SIGNED_MAGNITUDE defines what pins are used to control the motors: 4=DIR1, 5=EN1, 6=EN2, 7=DIR2;

//CytronMD rightMotor(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
//CytronMD leftMotor(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

int en_1 = 5;
int en_2 = 6;

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

const double KP=27;  // The proportional gain of the PID controller
const double KI=15;  // The integral gain of the PID controller
const double KD=0;  // The derivative gain of the PID controller
const double dt = 0.05;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(50000); // The processor executes timerInterrupt every 50ms
  
  Serial.begin(9600); // open the serial monitor at a rate of 250000bit/s
  
  pinMode(en_1, OUTPUT);  // motor 1 PWM speed
  pinMode(en_2, OUTPUT);  // motor 2 PWM speed

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  myservo.attach(9);  // the signal wire of the servo is attached to digital pin 9
}

void loop() {
  rightSetpoint = 5;
  leftSetpoint = 5;
  Serial.print("rightSpeed: "); Serial.print(rightSpeed);
  Serial.print("\tleftSpeed: "); Serial.print(leftSpeed);
  Serial.print("\tangleZ: ");
  mpu6050.update();
  Serial.println(mpu6050.getAngleZ());
  delay(50);
}

void timerInterrupt(){  //change this to double and return the left and right PWMs?
  // For the right motor
  rightEncoder = rightEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeRightEncoder = rightEncoder - prevRightEncoder;
  prevRightEncoder = rightEncoder;
  changeRightCentimeters = (float)changeRightEncoder/(float)TICKS_PER_CM_right; // convert encoder ticks to distance measurements
  rightSpeed = changeRightCentimeters/0.05; // divide the distance measurements by 50ms and get speed
  //Update the PID terms
  rightError = rightSetpoint - rightSpeed;
  changeInErrorRight = rightError - prevErrorRight;
  prevErrorRight = rightError;
  rightErrorSum += rightError;

  rightPID = KP*rightError + KI*rightErrorSum*dt + changeInErrorRight*KD/dt;
  rightPWM = velToPWMRight(rightSetpoint) + rightPID;
  
  //motor1.control(rightPWM,0); //One attempt to control the right motor using the Shield2AMotor library
  //rightMotor.setSpeed(rightPWM);  //Another attempt to control the right motor using the CytronMotorDriver library
  //digitalWrite(en_1,rightPWM);  //Another attempt to control the right motor using digitalWrite to the enable pin on the arduino

  // For the left motor
  leftEncoder = leftEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeLeftEncoder = leftEncoder - prevLeftEncoder;
  prevLeftEncoder = leftEncoder;
  changeLeftCentimeters = (float)changeLeftEncoder/(float)TICKS_PER_CM_left;
  leftSpeed = changeLeftCentimeters/0.05;
  //Update PID terms
  leftError = leftSetpoint - leftSpeed;
  changeInErrorLeft = leftError - prevErrorLeft;
  prevErrorLeft = leftError;
  leftErrorSum += leftError;

  leftPID = KP*leftError + KI*leftErrorSum*dt + changeInErrorLeft*KD/dt;
  leftPWM = velToPWMLeft(leftSetpoint) + leftPID;
  
  //motor2.control(0,leftPWM); //One attempt to control the left motor using the Shield2AMotor library
  //leftMotor.setSpeed(leftPWM);  //Another attempt to control the left motor using the CytronMotorDriver library
  //digitalWrite(en_2,leftPWM);  //Another attempt to control the left motor using digitalWrite to the enable pin on the arduino

  motors.control(-rightPWM,-leftPWM);
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
/*
void forward(int timer){
  motors.control(rightPWM, leftPWM);
  delay(timer);  // the time in this function is an arbitrary value
}

double desiredOrientation(double x_block, double y_block, double x_robot, double y_robot){
  double y = y_block - y_robot;
  double x = x_block - x_robot;
  double theta = atan2(y, x); // desired angle, heading
  return theta;
}

void turnRobot(double theta, int rightSetpoint, int leftSetpoint){
  if(theta > 0){
    double omega = 1; // confused on how to find angular velocity
    double time1 = theta / omega;
    motors.control(0, leftPWM);
    delay(time1*1000);
    motors.control(0, 0); // change setPoint instead
  }
  if(theta < 0){
    double omega = 1; // confused on how to find angular velocity
    double time1 = theta / omega;
    motors.control(rightPWM, 0);
    delay(time1*1000);
    motors.control(0, 0);
  }
}

double travelDistance(double x_block, double y_block, double x_robot, double y_robot){
  double a = abs(x_block - x_robot);
  double b = pow(a, 2);
  double c = abs(y_block - y_robot);
  double d = pow(c, 2);
  double hyp = sqrt(b + d);
  return hyp;
}
// use an if statement
// if not, drive forwards
// if angle is wrong, rotate
// if distance is hyp-0.5, stop

void moveRobot(double hyp, int rightSetpoint, int leftSetpoint){
  double time1 = (hyp - 0.5) / rightSetpoint;
  rightPWM = velToPWM(rightSetpoint) + rightPID;
  leftPWM = velToPWM(leftSetpoint) + leftPID;
  motors.control(rightPWM, leftPWM);
  delay(time1*1000);
  motors.control(0, 0);
}

void gripObject(){
  // Open the gripper
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  // Move the robot closer to the block
  moveRobot(1.0, 5, 5);

  // Close the gripper
  for (pos = 60; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void rotate(){
  motors.control(rightPWM, 0);
  delay(t*1000);  // need to use trial and error to find the time it would take to rotate the robot 180 deg
}

travelDistance(double curr_x, double curr_y, double x_robot, double y_robot);

moveRobot(hyp+0.5, rightSetpoint, leftSetpoint);

left_setpoint = forwards_speed - angular_velocity
right_setpoint = forwards_speed + angular_velocity
*/
