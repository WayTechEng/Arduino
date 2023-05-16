/* Get tilt angles on X and Y, and rotation angle on Z
   Angles are given in degrees

   License: MIT
*/

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { }

  Serial.println(F("While calculating the offsets value, do not move the MPU6050 sensor"));
  delay(1000);
  // mpu.upsideDownMounting = true; // Comment on this line if MPU6050 is installed backwards
  mpu.calcOffsets();
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();

//  if ((millis() - timer) > 10) { // print data every 10m
//    timer = millis();
//  }    
  Serial.print("\tY : ");
  Serial.print(mpu.getAngleY());
  Serial.print("90\xc2\xb0\n");

  
  delay(10);
  
}
