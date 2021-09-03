#include "Shield2AMotor.h"
#include <Encoder.h>
#include <TimerOne.h> // Florentin uses a DueTimer library but since we don't have a Due, this works with a Mega. 

const int TICKS_PER_CM_right = 281;  // the number of encoder counts per cm. obtained from phase A code
const int TICKS_PER_CM_left = 281;  // the number of encoder counts per cm. obtained from phase A code

const byte rightChannelA = 19;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte rightChannelB = 18;  // ... this is for the motor on the right.

const byte leftChannelA = 3;  // defines the interrupt pins on the mega that the encoders are connected to...
const byte leftChannelB = 2;  // ... this is for the motor on the left.

Encoder leftEnc(leftChannelA, leftChannelB);  // using the encoder library, the left encoder is defined with it's pins.
Encoder rightEnc(rightChannelA, rightChannelB); // using the encoder library, the right encoder is defined with it's pins.

Shield2AMotor motors(SIGNED_MAGNITUDE); // used to control the two motors
//SIGNED_MAGNITUDE defines what pins are used to control the motors: 4=DIR1, 5=EN1, 6=EN2, 7=DIR2;

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

const double dt = 0.05;

//int bluePower = 8;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  Timer1.attachInterrupt(timerInterrupt); // from the TimerOne.h library, this calls the timerInterrupt function
  Timer1.initialize(dt*1000000); // The processor executes timerInterrupt every 50ms
  
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s

//  pinMode(bluePower, OUTPUT);
//  digitalWrite(bluePower, HIGH);
}

void loop() {
  for(i=0; i<100; i+=5){  // the for loop increases the duty cycle of the PWM signal sent to only one motor in this case
    motors.control(i,i);  // the motor.control is a method of the Shield2AMotor library that controls the speed of the motor with a specified PWM signal
    delay(2000);  // delay of 2000ms since it takes the motors sometime to get to the desired speed
  }
}

void timerInterrupt(){
  rightEncoder = rightEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeRightEncoder = rightEncoder - prevRightEncoder;
  prevRightEncoder = rightEncoder;
  changeRightCentimeters = (float)changeRightEncoder/(float)TICKS_PER_CM_right; // convert encoder ticks to distance measurements
  rightSpeed = changeRightCentimeters/dt; // divide the distance measurements by 50ms and get speed
  
  leftEncoder = leftEnc.read(); // leftEncoder stores the number of encoder reads since the motor was started
  changeLeftEncoder = leftEncoder - prevLeftEncoder;
  prevLeftEncoder = leftEncoder;
  changeLeftCentimeters = (float)changeLeftEncoder/(float)TICKS_PER_CM_left;
  leftSpeed = changeLeftCentimeters/dt;

  Serial.print(i); Serial.print(":\t");
  Serial.print("right speed:"); Serial.print(rightSpeed); Serial.print("\t");
  Serial.print("left speed:"); Serial.println(leftSpeed);
}
