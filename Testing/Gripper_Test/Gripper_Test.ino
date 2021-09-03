#include <Servo.h>

Servo myServo;
double pos = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.attach(9);
  pos = 0.0;
  myServo.write(pos);
  Serial.println("Gripper starts open");
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("Close and open the gripper");
  delay(8000);
  closeGripper();
  delay(8000);
  openGripper();
}

void closeGripper(){
  pos = 62.0;
  myServo.write(pos);
  return 0;
}

void openGripper(){
  pos = 0.0;
  myServo.write(pos);
  return 0;
}
