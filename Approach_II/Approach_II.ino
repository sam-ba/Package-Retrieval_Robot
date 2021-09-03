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
volatile double u = 350;  // from openCV: camera x coordinate of the drop off location
volatile double v = 210;  // from openCV: camera y coordinate of the drop off location 
volatile double p = 500;  // from openCV: camera x coordinate of the second block
volatile double q = 350;  // from openCV: camera y coordinate of the second block

volatile double roboX = 0.0; // from openCV: actual x coordinate of the robot
volatile double roboY = 0.0; // from openCV: actual y coordinate of the robot
volatile double blockX = 0.0; // from openCV: actual x coordinate of the block
volatile double blockY = 0.0; // from openCV: actual y coordinate of the block
volatile double dropX = 0.0; // from openCV: actual x coordinate of the drop off location
volatile double dropY = 0.0; // from openCV: actual y coordinate of the drop off location
volatile double blockX2 = 0.0; // from openCV: actual x coordinate of the second block
volatile double blockY2 = 0.0; // from openCV: actual y coordinate of the second block

// for the gripper
volatile double pos;  // stores the position of the gripper

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

//double coordinatesX[6] = {roboX, blockX, dropX, blockX2, dropX, roboX};  // an array to store x coordinates
//double coordinatesY[6] = {roboY, blockY, dropY, blockY2, dropY, roboY};  // an array to store y coordinates

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  myServo.attach(9);
  pos = 0.0;
  myServo.write(pos);

  Serial.print("roboX: "); Serial.print(roboX); Serial.print("\troboY: "); Serial.println(roboY);
  Serial.print("blockX: "); Serial.print(blockX); Serial.print("\tblockY: "); Serial.println(blockY);
  Serial.print("x diff: "); Serial.print(x_diff); Serial.print("\ty diff: "); Serial.println(y_diff);
}

void loop() {
  // put your main code here, to run repeatedly:
//  char input = Serial.read(); //reads the information received on the serial monitor from python
//  
//  switch (input) {      //uses input for the switch case statement
//    case ('x'):     //indicates the start of the X coordinate from python
//      x=Serial.parseFloat();  //assigns the first float value to x before the next character.
//      break;
//    case ('y'):     //indicates the start of the Y coordinates read from python
//      y = Serial.parseFloat();  //passes the next float value to y
//      break;
//    case ('p'):     //indicates the start of the p coordinates read from python
//      p = Serial.parseFloat();  //passes the next float value to b
//      break;
//    case ('q'):     //indicates the start of the q coordinates read from python
//      q = Serial.parseFloat();  //passes the next float value to b
//      break;
//    case ('a'):     //indicates the start of the a coordinates read from python
//      a = Serial.parseFloat();  //passes the next float value to a
//      break;
//    case ('b'):     //indicates the start of the b coordinates read from python
//      b = Serial.parseFloat();  //passes the next float value to b
//      break;
//    case ('u'):     //indicates the start of the u coordinates read from python
//      u = Serial.parseFloat();  //passes the next float value to u
//      break;
//    case ('v'):     //indicates the start of the v coordinates read from python
//      v = Serial.parseFloat();  //passes the next float value to v
//      break;
//  }
//
//  //To print on the python console instead of the arduino serial port
//   Serial.print("Yellow"); Serial.print(" "); Serial.print(x); Serial.print(" "); Serial.print(y);    
//   Serial.print(" ");   //for format purposes
//   Serial.print("Blue"); Serial.print(" "); Serial.print(p); Serial.print(" "); Serial.print(q);
//   Serial.print(" ");
//   Serial.print("Robo"); Serial.print(" ");Serial.print(a); Serial.print(" "); Serial.println(b);    //prints the y coordinate read from python back to python

  
  roboX = pixToCoordX(a);
  roboY = pixToCoordY(b);

  blockX = pixToCoordX(x) - 8;
  blockY = pixToCoordY(y);

  dropX = pixToCoordX(u) - 8;
  dropY = pixToCoordY(v);

  blockX2 = pixToCoordX(p) - 8;
  blockY2 = pixToCoordY(q);

  double coordinatesX[6] = {roboX, blockX, dropX, blockX2, dropX, roboX};  // an array to store x coordinates
  double coordinatesY[6] = {roboY, blockY, dropY, blockY2, dropY, roboY};  // an array to store y coordinates

//  for(int i = 0; i < 6; i++){
//    y_diff = diffInY(coordinatesY[i+1], coordinatesY[i]);
//    x_diff = diffInX(coordinatesX[i+1], coordinatesX[i]);
//
//  Serial.print("Y2: "); Serial.print(coordinatesY[i+1]);  Serial.print("\t Y1: ");  Serial.println(coordinatesY[i]);
//  Serial.print("X2: "); Serial.print(coordinatesX[i+1]);  Serial.print("\t X1: ");  Serial.println(coordinatesX[i]);

  y_diff = diffInY(blockY, roboY);
  x_diff = diffInX(blockX, roboX);
  Serial.print("x diff: "); Serial.print(x_diff); Serial.print("\ty diff: "); Serial.println(y_diff);

  delay(5000);
  
  while(abs(y_diff) - distTravel > 0 && y_diff > 0){
    distTravel = distTrav();
    Serial.print("Forward");  Serial.print("\t distance travelled: "); Serial.println(distTravel);
    forwardSpeed = 17.0;
    angularVelocity = 0.0;
    speedControl();
  }

  while(abs(y_diff) - distTravel > 0 && y_diff < 0){
    distTravel = distTrav();
    Serial.println("Backward"); Serial.print("\t distance travelled: "); Serial.println(distTravel);
    forwardSpeed = -17.0;
    angularVelocity = 0.0;
    speedControl();
  }
  
  while(abs(abs(y_diff) - distTravel) < 0.5){
    Serial.print("Stop and rotate");
    forwardSpeed = 0.0;
    angularVelocity = 0.0;
    speedControl();
    distTravel = distTrav() - distTravel; // distTravel = 0.0; // totalDist = 0;  changeDist = 0; distLeft = 0; distRight = 0;// reset the distance travelled
    Serial.print("\t distance travelled: "); Serial.println(distTravel);

    while(x_diff > 0){
      Serial.println("Turn right");
      forwardSpeed = 0.0;
      angularVelocity = 10.0;
      speedControl();
      delay(2000); // the time it takes for the robot to make a 90 deg turn based on the angular velocity is gotten from trial and error
      distTravel = distTrav() - distTravel; 
      while(abs(x_diff) - distTravel > 0){
         distTravel = distTrav();
         Serial.print("Forward"); Serial.print("\tdistance travelled: "); Serial.println(distTravel);
         forwardSpeed = 17.0;  // move forward until 8cm away from the block
         angularVelocity = 0.0;
         speedControl();
      }
      while(abs(x_diff) - distTravel <= 0){
          distTravel = distTrav();
          Serial.print("Stop and grip"); 
          forwardSpeed = 0.0;
          angularVelocity = 0.0;
          speedControl();
//          closeGripper();
      }
    }

    while(x_diff < 0){
      Serial.println("Turn left");
      forwardSpeed = 0.0;
      angularVelocity = -10.0;
      speedControl();
      delay(2000); // the time it takes for the robot to make a 90 deg turn based on the angular velocity is gotten from trial and error
      distTravel = distTrav() - distTravel;
      Serial.print("distance travelled: "); Serial.println(distTravel);
      while(abs(x_diff) - distTravel > 0){
         distTravel = distTrav();
         Serial.print("Forward"); Serial.print("\tdistance travelled: "); Serial.println(distTravel);
         forwardSpeed = 17.0;  // move forward until 8cm away from the block
         angularVelocity = 0.0;
         speedControl();
      }
      do{   // the robot stops and closes or opens the gripper before realizing that it is at the goal and breaks the loop
         distTravel = distTrav();
         Serial.println("Stop and grip"); 
         forwardSpeed = 0.0;
         angularVelocity = 0.0;
         speedControl();
//         if(pos = 0){
//          closeGripper();
//         }
//         else if(pos = 62.0){
//          openGripper();
//         }
      }while(abs(x_diff) - distTravel > 0);
    x_diff = 0;
    }   
  }
  
  //}
}
  
double diffInY(double y_block, double y_robot){
  double y = y_block - y_robot;
  return y;
}

double diffInX(double x_block, double x_robot){
  double x = x_block - x_robot;
  return x;
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
void openGripper(){
  pos = 0.0;
  myServo.write(pos);
}

void closeGripper(){
  pos = 62.0;
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
