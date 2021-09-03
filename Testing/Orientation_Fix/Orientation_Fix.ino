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

Servo myServo;  // object of the servo library to control the gripper

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

//openCV parameters
volatile double a = 611; // from openCV: camera x coordinate of the robot
volatile double b = 439; // from openCV: camera y coordinate of the robot
volatile double x = 300; // from openCV: camera x coordinate of the yellow block
volatile double y = 289; // from openCV: camera y coordinate of the yellow block
volatile double roboX = 0.0; // from openCV: actual x coordinate of the robot
volatile double roboY = 0.0; // from openCV: actual y coordinate of the robot
volatile double blockX = 0.0; // from openCV: actual x coordinate of the block
volatile double blockY = 0.0; // from openCV: actual y coordinate of the block

//For the function for getting orientation for the encoders
volatile double delta_left;
volatile double delta_right;
volatile double delta_orientation;
volatile double delta_distance;
volatile double orientation;
#define WHEELBASE 13.8  // in cm

//for the navigation of the robot
volatile double theta = 0.0;  // stores the orientation of the robot
volatile double hyp = 0.0;  // stores the distance from the robot to the target position
volatile double angularVelocity = 0.0;  //  the angular velocity for turning functions
volatile double forwardSpeed = 0.0; // the speed to move the robot
volatile double distTravel = 0.0; // the actual distance travelled by the robot from the encoders
volatile double distLeft = 0.0; // the distance travelled by the left wheel measured from the encoders
volatile double distRight = 0.0;  // the distance travelled by the right wheel measured from the encoders
volatile double totalDist = 0.0;  // the total distance travelled by the robot
volatile double changeDist = 0.0; // stores the change in distance travelled by the robot every time the timerInterrupt fires

// for the gripper
volatile double pos;  // stores the position of the gripper

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(dt*1000000); // The processor executes timerInterrupt every 50ms

  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  myServo.attach(9);
  pos = 0.0;
  myServo.write(pos);
}

void loop() {
  // put your main code here, to run repeatedly:
//  if(Serial.available() > 0){
//  char input = Serial.read(); //reads the information received on the serial monitor from python
//  
//    switch (input) {      //uses input for the switch case statement
//      case ('x'):     //indicates the start of the X coordinate from python
//      x=Serial.parseFloat();  //assigns the first float value to x before the next character.
//      break; 
//      case ('y'):     //indicates the start of the Y coordinates read from python
//      y = Serial.parseFloat();  //passes the next float value to y
//      break;
//      case ('a'):     //indicates the start of the a coordinates read from python
//      a = Serial.parseFloat();  //passes the next float value to a
//      break;
//     case ('b'):     //indicates the start of the b coordinates read from python
//      b = Serial.parseFloat();  //passes the next float value to b
//      break;
//    }
//  }

  roboX = pixToCoordX(a);
  roboY = pixToCoordY(b);

  blockX = pixToCoordX(x);
  blockY = pixToCoordY(y);

  Serial.print("desired orientation: ");  Serial.print(theta);
  Serial.print("\tdistance to goal :");  Serial.print(hyp);
  Serial.print("\torientation: "); Serial.print(orientation);
  Serial.print("\tdist trav: ") ; Serial.println(distTravel);
   if(abs((orientation - theta)) > 0.0008){
      if((orientation - theta) > 0){
        Serial.println("Right");
      } 
      else{
        Serial.println("Left");
      }
   }
   else if(orientation == theta || (orientation - theta) < 0.0008){
      if(distTravel < (hyp-8)){
        Serial.println("Forward");
      }
      else{
        Serial.println("Stop to grip object");
      }
   }
}

void timerInterrupt(){ 
  theta = desiredOrientation(blockX, blockY, roboX, roboY); // calculate the angle that the robot should turn to face the block in a straight line
  hyp = travelDistance(blockX, blockY, roboX, roboY);
  distTravel = distTrav();
  orientation = getOrientation(); // the current orientation of the robot
  adjustOrientation();  // change the orientation of the robot until it is at the desired
  leftSetpoint = forwardSpeed - angularVelocity;  // this updates the speed of the left motor
  rightSetpoint = forwardSpeed + angularVelocity; // this updates the speed of the right motor
  speedControl();
}

double desiredOrientation(double x_block, double y_block, double x_robot, double y_robot){
  double y = y_block - y_robot;
  double x = x_block - x_robot;
  double theta = atan2(y, x); // desired angle, heading
//  theta = theta * (180/PI); // convert theta to degrees since the mpu also returns degrees
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

double distTrav(){
  rightEncoder = rightEnc.read(); // rightEncoder stores the number of encoder reads since the motor was started
  changeRightEncoder = rightEncoder - prevRightEncoder;
  prevRightEncoder = rightEncoder;
  distRight = (float)changeRightEncoder/(float)TICKS_PER_CM_right; 

  leftEncoder = leftEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeLeftEncoder = leftEncoder - prevLeftEncoder;
  prevLeftEncoder = leftEncoder;
  distLeft = (float)changeLeftEncoder/(float)TICKS_PER_CM_left;
  
//  distLeft = leftEnc.read()/TICKS_PER_CM_left;
//  distRight = rightEnc.read()/TICKS_PER_CM_right;

  changeDist = (distLeft - distRight)/2;
  totalDist += changeDist;
  return totalDist;
}

double getOrientation(){
  rightEncoder = rightEnc.read(); // rightEncoder stores the number of encoder reads since the motor was started
  changeRightEncoder = rightEncoder - prevRightEncoder;
  prevRightEncoder = rightEncoder;
  changeRightCentimeters = (float)changeRightEncoder/(float)TICKS_PER_CM_right; // convert encoder ticks to distance measurements

  leftEncoder = leftEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeLeftEncoder = leftEncoder - prevLeftEncoder;
  prevLeftEncoder = leftEncoder;
  changeLeftCentimeters = (float)changeLeftEncoder/(float)TICKS_PER_CM_left;

  delta_orientation = (changeLeftCentimeters - changeRightCentimeters)/WHEELBASE;
  orientation += delta_orientation;
  return orientation;

//  delta_left = leftEnc.read()/TICKS_PER_CM_left - leftEncoder;
//  delta_right = rightEnc.read()/TICKS_PER_CM_right - rightEncoder;
//  leftEncoder = leftEnc.read()/TICKS_PER_CM_left;
//  rightEncoder = rightEnc.read()/TICKS_PER_CM_right;
//
//  delta_orientation = (delta_left - delta_right)/WHEELBASE;
//  orientation += delta_orientation;
//  return orientation;
}

int speedControl(){
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

void adjustOrientation(){
//   if(orientation == theta || (orientation - theta) < 0.0008){
//      if(distTravel < (hyp-8)){
//        forwardSpeed = 25.0;  // move the robot if the distance to target is greater than 9cm
//        angularVelocity = 0.0;   // don't rotate. Just move straight
//      }
//      else{
//        forwardSpeed = 0.0; // stop the robot if the robot is 9cm or less away from the target
//        closeGripper();     // deploy the gripper once the robot is less than 9cm away from the target
//      }
//   }
//   else{
//      if((orientation - theta) > 0.0008){
//        angularVelocity = 10.0; // if orientation is greater than theta, then turn right
//      } 
//      else{
//        angularVelocity = -10.0;  // if orientaion is less than theta then turn left
//      }
//   }

   if(abs((orientation - theta)) > 0.0008){
      if((orientation - theta) > 0){
        angularVelocity = 10.0; // if orientation is greater than theta, then turn right
      } 
      else{
        angularVelocity = -10.0;  // if orientaion is less than theta then turn left
      }
   }
   else if(orientation == theta || (orientation - theta) < 0.0008){
      if(distTravel < (hyp-8)){
        forwardSpeed = 25.0;  // move the robot if the distance to target is greater than 9cm
        angularVelocity = 0.0;   // don't rotate. Just move straight
      }
      else{
        forwardSpeed = 0.0; // stop the robot if the robot is 9cm or less away from the target
        closeGripper();     // deploy the gripper once the robot is less than 9cm away from the target
      }
   }
}

void closeGripper(){
  pos = 62.0;
  myServo.write(pos);
}

void openGripper(){
  pos = 0.0;
  myServo.write(pos);
}

double pixToCoordX(double a){
  double x = 0.0491*a + 59.818;
  return x;
}

double pixToCoordY(double b){
  double y = -0.1928*b + 130.8;
  return y;
}
