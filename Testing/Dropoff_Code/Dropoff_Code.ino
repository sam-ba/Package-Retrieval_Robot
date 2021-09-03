#include <Servo.h>  // Used to control the gripper

//motor controls
//to control left motor direction and speed
const int IN1 = 6;  
const int IN2 = 7;
const int ENA = 3;
//to control right motor direction and speed
const int IN3 = 4;
const int IN4 = 5;
const int ENB = 2;

//Servo parameters
Servo myServo;  // object of the servo library to control the gripper
double pos = 0.0; // used to store the position of the gripper

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");

  //myServo.attach(9);
  //pos = 0.0;
  //myServo.write(pos);

  pinMode(IN1,OUTPUT);   
  pinMode(IN2,OUTPUT);  
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT); 
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  //Motor Control
// digitalWrite(IN1, LOW);  
// digitalWrite(IN2, HIGH);
// digitalWrite(IN3, HIGH);  
// digitalWrite(IN4, LOW); 
// analogWrite(ENA, 180);
// analogWrite(ENB, 185);

}

void loop() {
  // put your main code here, to run repeatedly:
  forward(5474);
  stopp();
  turn90Right(697);//697
  stopp();
  forward(825); //725
  stopp();
//  turn180(2000);
//  stopp();
//  reverse(999);
//  //stopp();
//  reverse(4999);
//  stopp();
//  delay(3000);
//  forward(5000);
//  stopp();
//  
}


void forward(int dt){
 digitalWrite(IN1, LOW);  
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, HIGH);  
 digitalWrite(IN4, LOW); 
 analogWrite(ENA, 180);
 analogWrite(ENB, 171);
 delay(dt);
}

void reverse(int dt){
 digitalWrite(IN1, HIGH);  
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);  
 digitalWrite(IN4, HIGH); 
 analogWrite(ENA, 180);
 analogWrite(ENB, 183);
 delay(dt);
}

void stopp(){
 analogWrite(ENA, 0);
 analogWrite(ENB, 0);
 delay(3000);
}

void turn90Right(int dt){
  analogWrite(ENA, 195);
  analogWrite(ENB, 200);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(dt); 
 }

 void turn180(int dt){
  analogWrite(ENA, 195);
  analogWrite(ENB, 0);
  delay(dt);
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
