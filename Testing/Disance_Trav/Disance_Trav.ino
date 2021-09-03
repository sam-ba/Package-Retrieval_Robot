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

Shield2AMotor motors(SIGNED_MAGNITUDE);

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

const double KP=0.048;  // The proportional gain of the PID controller. the output should oscillate about the setpoint
const double KD=0.005;  // The derivative gain of the PID controller
const double KI=0;  // The integral gain of the PID controller
const double dt = 0.05; // let the timer be 500ms for the interrupt

// for the navigation
volatile double angularVelocity = 0.0;  //  the angular velocity for turning functions
volatile double forwardSpeed = 0.0; // the speed to move the robot
volatile double y_diff = 0.0; // the difference between the y coord of the robot and block
volatile double x_diff = 0.0; // the difference between the x coord of the robot and block
volatile double distTravel = 0.0; // the actual distance travelled by the robot from the encoders
volatile double distLeft = 0.0; // the distance travelled by the left wheel measured from the encoders
volatile double distRight = 0.0;  // the distance travelled by the right wheel measured from the encoders
volatile double totalDist = 0.0;  // the total distance travelled by the robot
volatile double changeDist = 0.0; // stores the change in distance travelled by the robot every time the timerInterrupt fires

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Code starts:");
}

void loop() {
  // put your main code here, to run repeatedly:
  forwardSpeed = 17.0;
  angularVelocity = 0.0;
  speedControl();
  distTravel = distTrav();
  Serial.println(distTravel);
  delay(10000);
  
  Serial.println("Reset variables");
  totalDist = 0;  changeDist = 0; distLeft = 0; distRight = 0;  distTravel = 0; rightEnc.write(0);  leftEnc.write(0); prevRightEncoder = 0; 
  prevLeftEncoder = 0;

  distTravel = distTrav();
  Serial.println(distTravel);
}

double distTrav(){
  rightEncoder = rightEnc.read(); // rightEncoder stores the number of encoder reads since the motor was started
  changeRightEncoder = rightEncoder - prevRightEncoder;
  prevRightEncoder = rightEncoder;
  distRight = (float)changeRightEncoder/(float)TICKS_PER_CM_right; 

  leftEncoder = leftEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeLeftEncoder = leftEncoder - prevLeftEncoder;
  prevLeftEncoder = leftEncoder;
  distLeft = (float)changeLeftEncoder/(float)TICKS_PER_CM_left;
  
  changeDist = (distLeft - distRight)/2;
  totalDist += changeDist;
  return totalDist;
}

int speedControl(){
  leftSetpoint = forwardSpeed - angularVelocity;  // this updates the speed of the left motor
  rightSetpoint = forwardSpeed + angularVelocity; // this updates the speed of the right motor
  
  // For the right motor
  rightEncoder = rightEnc.read(); // rightEncoder stores the number of encoder reads since the motor was started
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
