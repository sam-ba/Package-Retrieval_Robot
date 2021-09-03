#include "Shield2AMotor.h"  // Used to define the motor object
#include <Encoder.h>  // Used to define the encoders connected to the motors
#include <TimerOne.h> // Used for the timer interrupt 
#include <Servo.h>  // Used to control the gripper

const int TICKS_PER_CM_right = 281;  // the number of encoder counts per cm. obtained from phase A code
const int TICKS_PER_CM_left = 281;  // the number of encoder counts per cm. obtained from phase A code

const byte rightChannelA = 19;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte rightChannelB = 18;  // ... this is for the motor on the right.

const byte leftChannelA = 3;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte leftChannelB = 2;  // ... this is for the motor on the left.

Encoder leftEnc(leftChannelA, leftChannelB);  // using the encoder library, the left encoder is defined with it's pins.
Encoder rightEnc(rightChannelA, rightChannelB); // using the encoder library, the right encoder is defined with it's pins.

Shield2AMotor motors(SIGNED_MAGNITUDE); // used to control both motors 

#define PI 3.1415926535897932384626433832795  // define the math constant pi in rad
double pi_deg = 180.0;
double pi_rad = 3.14;

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
const double dt = 0.05; // let the timer be 500ms for the interrupt

//For the function for getting orientation for the encoders
volatile double delta_left;
volatile double delta_right;
volatile double delta_orientation;
volatile double delta_distance;
volatile double orientation;
#define WHEELBASE 13.8  // in cm

volatile double forwardSpeed;
volatile double angularVelocity;

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt);
  Timer1.initialize(dt*1000000);
  
  Serial.begin(9600);
  Serial.println("Code starts: ");
}

void loop() {
  // put your main code here, to run repeatedly:
//  forwardSpeed = 0.0;
//  angularVelocity = -10.0;
  Serial.print("orientation: "); Serial.println(orientation);
}

void timerInterrupt(){
  orientation = getOrientation();
  leftSetpoint = 25.0; //forwardSpeed - angularVelocity;
  rightSetpoint = 0.0;  //forwardSpeed + angularVelocity;
  speedControl();
}

double getOrientation(){
  delta_left = leftEnc.read()/TICKS_PER_CM_left - leftEncoder;
  delta_right = rightEnc.read()/TICKS_PER_CM_right - rightEncoder;
  leftEncoder = leftEnc.read()/TICKS_PER_CM_left;
  rightEncoder = rightEnc.read()/TICKS_PER_CM_right;

  delta_orientation = (delta_left - delta_right)/WHEELBASE;
  orientation += delta_orientation;
  return orientation;
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
