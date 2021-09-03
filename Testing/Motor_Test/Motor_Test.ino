#include "Shield2AMotor.h"

Shield2AMotor motor1(SIGNED_MAGNITUDE);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Code starts");
}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.control(-150,-150);
}
