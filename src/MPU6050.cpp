/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
//#include "Netprint.h"
MPU6050 mpu(Wire);


void MPU6050_Init() {
  
  Wire.begin(9,46);         //初始化I2C总线-----------------------------------------------
  
  byte status = mpu.begin();
 Serial.print("MPU6050 status: ");

  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.print("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true,true); // 校准角速度和加速度-------------------------------------------
 Serial.print("Done!\n");
  
}

unsigned long timer = 0;

void MPU6050_check() {
  mpu.update();
  if (millis() - timer > 1000) { // 每秒打印一次
    Serial.print("TEMPERATURE: "); Serial.print(mpu.getTemp()); Serial.println();
    Serial.print("ACCELERO  X: "); Serial.print(mpu.getAccX());
    Serial.print("\tY: "); Serial.print(mpu.getAccY());
    Serial.print("\tZ: "); Serial.println(mpu.getAccZ());
    Serial.print("GYRO      X: "); Serial.print(mpu.getGyroX());
    Serial.print("\tY: "); Serial.print(mpu.getGyroY());
    Serial.print("\tZ: "); Serial.println(mpu.getGyroZ());
    Serial.print("ACC ANGLE X: "); Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: "); Serial.println(mpu.getAccAngleY());
    Serial.print("ANGLE     X: "); Serial.print(mpu.getAngleX());
    Serial.print("\tY: "); Serial.print(mpu.getAngleY());
    Serial.print("\tZ: "); Serial.println(mpu.getAngleZ());
    Serial.println("====================================================");
    timer = millis();
  }
}