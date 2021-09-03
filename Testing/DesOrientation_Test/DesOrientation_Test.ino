#include "Shield2AMotor.h"  // Used to define the motor object
#include <Encoder.h>  // Used to define the encoders connected to the motors
#include <TimerOne.h> // Used for the timer interrupt 
#include <Servo.h>  // Used to control the gripper

//openCV parameters
volatile double a = 611; // from openCV: camera x coordinate of the robot
volatile double b = 439; // from openCV: camera y coordinate of the robot
volatile double x = 300; // from openCV: camera x coordinate of the yellow block
volatile double y = 289; // from openCV: camera y coordinate of the yellow block
volatile double roboX = 0.0; // from openCV: actual x coordinate of the robot
volatile double roboY = 0.0; // from openCV: actual y coordinate of the robot
volatile double blockX = 0.0; // from openCV: actual x coordinate of the block
volatile double blockY = 0.0; // from openCV: actual y coordinate of the block

volatile double dt = 0.05;
volatile double theta;
volatile double hyp;

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(dt*1000000); // The processor executes timerInterrupt every 50ms

  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  Serial.print("roboX:"); Serial.print(roboX);  Serial.print("\troboY:") ;  Serial.println(roboY);
  Serial.print("blockX:"); Serial.print(blockX);  Serial.print("\tblockY:") ;  Serial.println(blockY);
}

void loop() {
  // put your main code here, to run repeatedly:
  roboX = pixToCoordX(a);
  roboY = pixToCoordY(b);

  blockX = pixToCoordX(x);
  blockY = pixToCoordY(y);
  Serial.print("desired orientation: ");  Serial.println(theta);
  Serial.print("travel distance :");  Serial.println(hyp);
}

void timerInterrupt(){ 
  theta = desiredOrientation(blockX, blockY, roboX, roboY); // calculate the angle that the robot should turn to face the block in a straight line
  hyp = travelDistance(blockX, blockY, roboX, roboY);
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

double pixToCoordX(double a){
  double x = 0.0491*a + 59.818;
  return x;
}

double pixToCoordY(double b){
  double y = -0.1928*b + 130.8;
  return y;
}
