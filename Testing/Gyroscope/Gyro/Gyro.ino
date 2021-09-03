//From Freenove kit with Teapot video tutorial library

#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 gyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

void setup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  Serial.begin(250000);
  Serial.println("Initializing I2C devices...");
  //Wire.begin();
  gyro.initialize();
  Serial.println("Testing device connections...");
  if(gyro.testConnection()){
    Serial.println("MPU6050 Connection successful");
  }
  else{
    Serial.println("MPU6050 Connection failed");
    while(1);
  }
  Serial.print("X,Y,Z offset :\t");
  Serial.print(gyro.getXAccelOffset()); Serial.print("\t");
  Serial.print(gyro.getYAccelOffset()); Serial.print("\t");
  Serial.print(gyro.getZAccelOffset()); Serial.print("\t");

  pinMode(LED_PIN, OUTPUT);
}

void loop(){
  gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  #ifdef OUTPUT_READABLE_ACCELGYRO
  Serial.print("a/g:\t");
  Serial.print((float)ax/16384); Serial.print("\t");
  Serial.print((float)ay/16384); Serial.print("\t");
  Serial.print((float)az/16384); Serial.print("\t");
  Serial.print((float)gx/131); Serial.print("\t");
  Serial.print((float)gy/131); Serial.print("\t");
  Serial.print((float)gz/131); //Serial.println("\t");
  #endif

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
