/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************************
 * DESCRIPTION:
 *
 * This example shows how to drive 2 motors using the PWM and DIR pins with
 * 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Arduino D3  - Motor Driver PWM 1 Input
 * Arduino D4  - Motor Driver DIR 1 Input
 * Arduino D9  - Motor Driver PWM 2 Input
 * Arduino D10 - Motor Driver DIR 2 Input
 * Arduino GND - Motor Driver GND
 *
 *
 * AUTHOR   : Kong Wai Weng
 * COMPANY  : Cytron Technologies Sdn Bhd
 * WEBSITE  : www.cytron.io
 * EMAIL    : support@cytron.io
 *
 *******************************************************************************/

// For turning the robot and controlling the speed using the cytron motor library

#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 5, 4);  // PWM 1 = Pin 5, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 6, 7); // PWM 2 = Pin 6, DIR 2 = Pin 7.

// this function moves both motors forward at 50% speed
void forward(int t){  
  motor1.setSpeed(map(50,0,100,0,255));
  motor2.setSpeed(map(50,0,100,0,255));
  delay(t);
}

/* this function turns the robot right by stopping one motor and 
 * moving the other.
 */
void right(int t){
  motor1.setSpeed(map(50,0,100,0,255));
  motor2.setSpeed(map(0,0,100,0,255));
  delay(t);
}
/* this function turns the robot left by stopping one motor and 
 * moving the other.
 */
void left(int t){
  motor1.setSpeed(map(0,0,100,0,255));
  motor2.setSpeed(map(50,0,100,0,255));
  delay(t);
}

// this function moves both motors backward at 50% speed
void backward(int t){
  motor1.setSpeed(-(map(50,0,100,0,255)));
  motor2.setSpeed(-(map(50,0,100,0,255)));
  delay(t);
}

// The setup routine runs once when you press reset.
void setup() {
  // nothing in the setup
}

// The loop routine runs over and over again forever.
void loop() {
  /*
  motor1.setSpeed(128);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
  delay(1000);
  
  motor1.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255);  // Motor 2 runs backward at full speed.
  delay(1000);

  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(1000);

  motor1.setSpeed(-128);  // Motor 1 runs backward at 50% speed.
  motor2.setSpeed(128);   // Motor 2 runs forward at 50% speed.
  delay(1000);
  
  motor1.setSpeed(-255);  // Motor 1 runs backward at full speed.
  motor2.setSpeed(255);   // Motor 2 runs forward at full speed.
  delay(1000);

  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(1000);
  */

  forward(5000);
  backward(3000);
  right(1000);
  left(1000);
  forward(1000);
}
