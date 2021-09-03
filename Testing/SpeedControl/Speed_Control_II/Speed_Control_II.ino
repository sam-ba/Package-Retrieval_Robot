#include "Shield2AMotor.h"
#include "CytronMotorDriver.h"
#include <Encoder.h>  // Used to define the encoders connected to the motors
#include <TimerOne.h> // Used for the timer interrupt 

const int TICKS_PER_CM_right = 287;  // the number of encoder counts per cm. obtained from phase A code
const int TICKS_PER_CM_left = 275;  // the number of encoder counts per cm. obtained from phase A code

const byte rightChannelA = 19;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte rightChannelB = 18;  // ... this is for the motor on the right.

const byte leftChannelA = 3;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte leftChannelB = 2;  // ... this is for the motor on the left.

Encoder leftEnc(leftChannelA, leftChannelB);  // using the encoder library, the left encoder is defined with it's pins.
Encoder rightEnc(rightChannelA, rightChannelB); // using the encoder library, the right encoder is defined with it's pins.

Shield2AMotor motor1(SIGNED_MAGNITUDE); // the motor on the right is defined as an object of the Shield2AMotor library.
Shield2AMotor motor2(SIGNED_MAGNITUDE); // the motor on the left is defined as an object of the Shield2AMotor library. 
//SIGNED_MAGNITUDE defines what pins are used to control the motors: 4=DIR1, 5=EN1, 6=EN2, 7=DIR2;
CytronMD rightMotor(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD leftMotor(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.
int en_1 = 5;
int en_2 = 6;

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

const double KP=500;  // The proportional gain of the PID controller
const double KI=450;  // The integral gain of the PID controller
const double KD=25;  // The derivative gain of the PID controller
const double dt = 0.05;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(50000); // The processor executes timerInterrupt every 50ms
  
  Serial.begin(250000); // open the serial monitor at a rate of 250000bit/s
  
  pinMode(en_1, OUTPUT);  // motor 1 PWM speed
  pinMode(en_2, OUTPUT);  // motor 2 PWM speed

  //Serial.print(rightSpeed); Serial.print(",\t");
  //Serial.println(leftSpeed);
}

void loop() {
  leftSetpoint = 10.0;
  rightSetpoint = 10.0;
  //Serial.print(rightSpeed);
  //Serial.print(",\t");
  //Serial.println(leftSpeed);
  delay(50);
}

void timerInterrupt(){
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
  digitalWrite(en_1,rightPWM);  //Another attempt to control the right motor using digitalWrite to the enable pin on the arduino

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
  digitalWrite(en_2,rightPWM);  //Another attempt to control the left motor using digitalWrite to the enable pin on the arduino
}

int velToPWMRight(int setpoint){
  if(setpoint > 0){
    int PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0615 - b*0.9505 + c*5.1442 - setpoint*5.2703 + 10.006;
    return PWM;
  }
  if(setpoint == 0){
    return 0;
  }
  if(setpoint > 0){
    int PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0615 - b*0.9505 + c*5.1442 - setpoint*5.2703 + 10.006;
    return -(PWM);
  }
}

int velToPWMLeft(int setpoint){
  if(setpoint > 0){
    int PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0178 - b*0.2512 + c*1.2369 + setpoint*1.6999 + 10.002;
    return PWM;
  }
  if(setpoint == 0){
    return 0;
  }
  if(setpoint > 0){
    int PWM = 0, a, b, c;
    a = pow(setpoint, 4);
    b = pow(setpoint, 3);
    c = pow(setpoint, 2);
    PWM = a*0.0178 - b*0.2512 + c*1.2369 + setpoint*1.6999 + 10.002;
    return -(PWM);
  }
}
