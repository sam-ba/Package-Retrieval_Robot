/*
 * This approach builds from phase B and finds the relationship between distance and time in a spreadsheet. that function is
 * used to find the time it takes to move any distance. 
 */

#include <Servo.h>  // Used to control the gripper

#define ENA 3
#define IN1 6
#define IN2 7

#define ENB 2
#define IN3 4
#define IN4 5

volatile double rpm = 0.0;  // the revolution per minute of the motors when the pin is set to HIGH
volatile double motorSpeed = 0.0; // the speed of the motors in centimeters 
volatile double timeElapsed = 0.0;  // stores the time elapsed using the millisecond counter
volatile double distTravel = 0.0; // stores the distance travelled by the robot

volatile double reqTime = 0.0;  // the time needed to cover a certain distance

Servo myServo;  // object of the servo library to control the gripper
double pos = 0.0; // used to store the position of the gripper

//openCV parameters
volatile double a = 580; // from openCV: camera x coordinate of the robot's starting location
volatile double b = 419; // from openCV: camera y coordinate of the robot's starting location
volatile double u = 74;  // from openCV: camera x coordinate of the red drop off location
volatile double v = 65;  // from openCV: camera y coordinate of the red drop off location 
volatile double c = 492;  // from openCV: camera x coordinate of the green drop off location
volatile double d = 245;  // from openCV: camera y coordinate of the green drop off location
volatile double x = 251; // from openCV: camera x coordinate of the first block
volatile double y = 284; // from openCV: camera y coordinate of the first block
volatile double p = 492;  // from openCV: camera x coordinate of the second block
volatile double q = 245;  // from openCV: camera y coordinate of the second block
volatile double i = 492;  // from openCV: camera x coordinate of the third block
volatile double j = 245;  // from openCV: camera y coordinate of the third block
volatile double m = 492;  // from openCV: camera x coordinate of the fourth block
volatile double n = 245;  // from openCV: camera y coordinate of the fourth block

volatile double roboX = 0.0; // from openCV: actual x coordinate of the robot's starting location
volatile double roboY = 0.0; // from openCV: actual y coordinate of the robot's starting location
volatile double dropX = 0.0; // from openCV: actual x coordinate of the red drop off location
volatile double dropY = 0.0; // from openCV: actual y coordinate of the red drop off location
volatile double dropX2 = 0.0; // from openCV: actual x coordinate of the green drop off location
volatile double dropY2 = 0.0; // from openCV: actual y coordinate of the green drop off location
volatile double blockX = 0.0; // from openCV: actual x coordinate of the firstblock
volatile double blockY = 0.0; // from openCV: actual y coordinate of the first block
volatile double blockX2 = 0.0; // from openCV: actual x coordinate of the second block
volatile double blockY2 = 0.0; // from openCV: actual y coordinate of the second block
volatile double blockX3 = 0.0; // from openCV: actual x coordinate of the second block
volatile double blockY3 = 0.0; // from openCV: actual y coordinate of the second block
volatile double blockX4 = 0.0; // from openCV: actual x coordinate of the second block
volatile double blockY4 = 0.0; // from openCV: actual y coordinate of the second block

volatile double y_diff = 0.0; // the difference between the y coord of the robot and block
volatile double x_diff = 0.0; // the difference between the x coord of the robot and block

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  myServo.attach(9);
  pos = 0.0;
  myServo.write(pos);
  
  pinMode(ENA,OUTPUT);  // the pin that controls the A channel
  pinMode(ENB,OUTPUT);  // the pin that controls the A channel
  pinMode(IN1,OUTPUT);  // the pin that controls motor 1 is set to output 
  pinMode(IN2,OUTPUT);  // the pin that controls motor 2 is set to output
  pinMode(IN3,OUTPUT);  // the pin that controls motor 3 is set to output
  pinMode(IN4,OUTPUT);  // the pin that controls motor 4 is set to output
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

//   timeElapsed = millis();
//   distTravel = motorSpeed * (timeElapsed/1000);

  roboX = pixToCoordX(a);
  roboY = pixToCoordY(b);

  dropX = pixToCoordX(u) - 8;
  dropY = pixToCoordY(v);

  dropX2 = pixToCoordX(c) - 8;
  dropY2 = pixToCoordY(d);
  
  blockX = pixToCoordX(x) - 8;
  blockY = pixToCoordY(y);

  blockX2 = pixToCoordX(p) - 8;
  blockY2 = pixToCoordY(q);
  
  blockX3 = pixToCoordX(i) - 8;
  blockY3 = pixToCoordY(j);

  blockX4 = pixToCoordX(m) - 8;
  blockY4 = pixToCoordY(n);


  Serial.print("roboX: "); Serial.print(roboX); Serial.print("\t roboY: "); Serial.println(roboY);
  Serial.print("blockX: "); Serial.print(blockX); Serial.print("\t blockY: "); Serial.println(blockY);
  Serial.print("dropX: "); Serial.print(dropX); Serial.print("\t dropY: "); Serial.println(dropY);
  Serial.print("blockX2: "); Serial.print(blockX2); Serial.print("\t blockY2: "); Serial.println(blockY2);

  double coordinatesX[10] = {roboX, blockX, dropX, blockX2, dropX, blockX3, dropX2, blockX4, dropX2, roboX};  // an array to store x coordinates
  double coordinatesY[10] = {roboY, blockY, dropY, blockY2, dropY, blockY3, dropY2, blockY4, dropY2, roboY};  // an array to store y coordinates

  for(int i = 0; i < 10; i++){
  y_diff = diffInY(coordinatesY[i+1], coordinatesY[i]);
  x_diff = diffInX(coordinatesX[i+1], coordinatesX[i]);

  Serial.print("Y2: "); Serial.print(coordinatesY[i+1]);  Serial.print("\t Y1: ");  Serial.println(coordinatesY[i]);
  Serial.print("X2: "); Serial.print(coordinatesX[i+1]);  Serial.print("\t X1: ");  Serial.println(coordinatesX[i]);

  delay(3000);

  while(y_diff > 0){
    Serial.print("Foward"); 
    forward();
    reqTime = getTime(abs(y_diff));
    delay(reqTime*1000);
    y_diff = 0;
  }

  while(y_diff < 0){
    Serial.print("Backward"); 
    backward();
    reqTime = getTime(abs(y_diff));
    delay(reqTime*1000);
    y_diff = 0;
  }

  while(abs(y_diff) < 0.05){
    Serial.println("Stop and Rotate"); 
    stopp();

    while(x_diff > 0){
      Serial.println("Turn Right");
      right();
      delay(2000);  // the time in this delay is how long the robot takes to make a 90 degree turn. it is gotten by trial and error
      Serial.println("Foward");
      forward();
      reqTime = getTime(abs(x_diff));
      delay(reqTime*1000);
      Serial.println("Stop and grip");
      // the code that makes the robot rotate, open the gripper, grip the block and rotate 180 deg again
      x_diff = 0;
    }

    while(x_diff < 0){
      Serial.println("Turn Left");
      left();
      delay(2000);  // the time in this delay is how long the robot takes to make a 90 degree turn. it is gotten by trial and error
      Serial.println("Foward");
      forward();
      reqTime = getTime(abs(x_diff));
      delay(reqTime*1000);
      Serial.println("Stop and grip");
      // insert the code that makes the robot rotate, open the gripper, grip the block and rotate 180 deg again
      x_diff = 0;
    }
  }
  }
}

void forward(){ 
  analogWrite(ENA,150); //enable L298n A channel
  analogWrite(ENB,150); //enable L298n B channel
  digitalWrite(IN1,HIGH); //set IN1 high level
  digitalWrite(IN2,LOW);  //set IN2 low level
  digitalWrite(IN3,LOW);  //set IN3 low level
  digitalWrite(IN4,HIGH); //set IN4 high level
  Serial.println("Forward");//send message to serial monitor
}

void backward(){
  analogWrite(ENA,150); //enable L298n A channel
  analogWrite(ENB,150); //enable L298n B channel
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Back");
}

void left(){
  analogWrite(ENA,150); //enable L298n A channel
  analogWrite(ENB,150); //enable L298n B channel
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
  Serial.println("Left");
}

void right(){
  analogWrite(ENA,150); //enable L298n A channel
  analogWrite(ENB,150); //enable L298n B channel
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Right");
}

void stopp(){
  analogWrite(ENA,150); //enable L298n A channel
  analogWrite(ENB,150); //enable L298n B channel
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

double diffInY(double y_block, double y_robot){
  double y = y_block - y_robot;
  return y;
}

double diffInX(double x_block, double x_robot){
  double x = x_block - x_robot;
  return x;
}

double pixToCoordX(double a){
  float g = 0, h = 0, k = 0;
  g = pow(a, 4);
  h = pow(a, 3);
  k = pow(a, 2);  
  double x = 0.0491*a + 59.818;
  return x;
}

double pixToCoordY(double b){
  float g = 0, h = 0, k = 0;
  g = pow(b, 4);
  h = pow(b, 3);
  k = pow(b, 2);  
  double y = -0.1928*b + 130.8;
  return y;
}

void openGrip(int dt){
  pos = 0.0;
  myServo.write(pos);
  delay(dt);
}

void closeGrip(int dt){
  pos = 80;
  myServo.write(pos);
  delay(dt);
}

double getTime(double distance){
  // insert function that relates distance and time
  // return time
}
