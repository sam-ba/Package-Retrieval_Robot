#include <Servo.h>  // Used to control the gripper

#define ENA 5
#define IN1 7
#define IN2 8

volatile double rpm = 0.0;  // the revolution per minute of the motors when the pin is set to HIGH

Servo myServo;  // object of the servo library to control the gripper
double pos = 0.0; // used to store the position of the gripper

//openCV parameters
volatile double a = 580; // from openCV: camera x coordinate of the robot's starting location
volatile double b = 419; // from openCV: camera y coordinate of the robot's starting location
volatile double x = 251; // from openCV: camera x coordinate of the first block
volatile double y = 284; // from openCV: camera y coordinate of the first block
volatile double u = 74;  // from openCV: camera x coordinate of the drop off location
volatile double v = 65;  // from openCV: camera y coordinate of the drop off location 
volatile double p = 492;  // from openCV: camera x coordinate of the second block
volatile double q = 245;  // from openCV: camera y coordinate of the second block

volatile double roboX = 0.0; // from openCV: actual x coordinate of the robot
volatile double roboY = 0.0; // from openCV: actual y coordinate of the robot
volatile double blockX = 0.0; // from openCV: actual x coordinate of the block
volatile double blockY = 0.0; // from openCV: actual y coordinate of the block
volatile double dropX = 0.0; // from openCV: actual x coordinate of the drop off location
volatile double dropY = 0.0; // from openCV: actual y coordinate of the drop off location
volatile double blockX2 = 0.0; // from openCV: actual x coordinate of the second block
volatile double blockY2 = 0.0; // from openCV: actual y coordinate of the second block

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  myServo.attach(9);
  pos = 0.0;
  myServo.write(pos);

  pinMode(IN1,OUTPUT);  // the pin that controls motor 1 is set to output 
  pinMode(IN2,OUTPUT);  // the pin that controls motor 2 is set to output
}

void loop() {
  // put your main code here, to run repeatedly:
  forward();
  delay(5000);
}

void forward(){
  digitalWrite(ENA, HIGH);  // enable the channel of the motor driver that the motors are connected to
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, HIGH);  
}

void backward(){
  digitalWrite(ENA, HIGH);  // enable the channel of the motor driver that the motors are connected to
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, LOW);  
}

void right(){
  digitalWrite(ENA, HIGH);  // enable the channel of the motor driver that the motors are connected to
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, HIGH);  
}

void left(){
  digitalWrite(ENA, HIGH);  // enable the channel of the motor driver that the motors are connected to
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, LOW);  
}
