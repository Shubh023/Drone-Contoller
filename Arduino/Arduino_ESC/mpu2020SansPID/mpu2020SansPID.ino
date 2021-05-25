#include "Wire.h"
#include <Servo.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
unsigned long timer = 0;
float roll = 0.0;
float leftval = 0;
float rightval = 0;

Servo left;
Servo right;

void setup() {
  //////////////////////// GYRO CALIBRATION //////////////////////////////
  Serial.begin(115200);
  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) { }
  delay(1000);
  mpu.calcOffsets();
  // Serial.print("GyroXoffset : ");
  // Serial.println(GyroXoffset);
  // delay(1000);

  //////////////////////// ESC CALIBRATION //////////////////////////////
  Serial.println("Starting ESCs Calibration");
  left.attach(6, 1000, 2000);  // PWM Pin connnection for left Motor 
  delay(1);
  left.write(0);
  
  right.attach(5, 930, 2000); // PWM Pin connnection for right Motor
  delay(1);
  right.write(0);

  
  // Mandatory Wait for ESC to be armed Correctly 
  delay(6000);
  Serial.println("Finished Calibration"); 
 }
 
 void loop() {
   mpu.update();
   if ((millis() - timer) > 0.1) {
       roll = mpu.getAngleX();
       Serial.print("roll : ");
       Serial.print(roll);
       leftval = map(roll, 25 , -25, 0, 30);
       rightval = map(roll, -25 ,25, 0, 15);
       left.write(leftval);
       right.write(rightval);
       Serial.println();
       timer = millis();
    }
 }
