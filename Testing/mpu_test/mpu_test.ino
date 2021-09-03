#include <MPU6050_tockn.h>  // Used for the gyroscope MPU6050
#include <Wire.h> //  Used for the gyroscope MPU6050

MPU6050 mpu6050(Wire);  // an object of the MPU6050 library used to control the gyroscope

#define PI 3.1415926535897932384626433832795  // define the math constant pi

const double pi_deg = 180.0;
volatile double tempAngle = 0.0;
volatile double currentAngle = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial monitor at a rate of 9600bit/s
  Serial.println("Code starts");
  
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // the gyro gets its offsets in the first 3 seconds of the robot getting powered
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  tempAngle = mpu6050.getAngleZ();
//  currentAngle = tempAngle;
  Serial.print("temp Angle: "); Serial.println(tempAngle);
//  if(tempAngle > pi_deg){  // find the shortest angle the robot should turn 
//    currentAngle = tempAngle - 2*pi_deg;  
//  }
//  if(tempAngle < -pi_deg){
//    currentAngle = tempAngle + 2*pi_deg;
//  }
  if(tempAngle > pi_deg){  // find the shortest angle the robot should turn 
    currentAngle = tempAngle - 2*pi_deg;  
  }
  else if(tempAngle < -pi_deg){
    currentAngle = tempAngle + 2*pi_deg;
  }
  else{
      currentAngle = tempAngle;
  }
  Serial.print("current Angle: "); Serial.println(currentAngle);
}
