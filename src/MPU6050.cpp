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
#include "Netprint.h"
MPU6050 mpu(Wire);


void MPU6050_Init() {
  
  Wire.begin(9,46);         //初始化I2C总线-----------------------------------------------
  
  byte status = mpu.begin();
 netPrintf("MPU6050 status: ");
//  netPrintf(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
 netPrintf("Calculating offsets, do not move MPU6050");
  delay(1000);
  mpu.calcOffsets(true,true); // 校准角速度和加速度-------------------------------------------
 netPrintf("Done!\n");
  
}

unsigned long timer = 0;

void MPU6050_check() {
  mpu.update();
  if (millis() - timer > 1000) { // 每秒打印一次
    netPrintf("TEMPERATURE: %.2f\n", mpu.getTemp());
    netPrintf("ACCELERO  X: %.2f\tY: %.2f\tZ: %.2f\n",
              mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
    netPrintf("GYRO      X: %.2f\tY: %.2f\tZ: %.2f\n",
              mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ());
    netPrintf("ACC ANGLE X: %.2f\tY: %.2f\n",
              mpu.getAccAngleX(), mpu.getAccAngleY());
    netPrintf("ANGLE     X: %.2f\tY: %.2f\tZ: %.2f\n",
              mpu.getAngleX(), mpu.getAngleY(), mpu.getAngleZ());
    netPrintf("=====================================================\n");
    timer = millis();
  }
}


//  if(millis() - timer > 1000){ // 打印输出信息
//    netPrintf(F("TEMPERATURE: "));netPrintf(mpu.getTemp());
//    netPrintf(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
//    netPrintf("\tY: ");Serial.print(mpu.getAccY());
//    netPrintf("\tZ: ");netPrintf(mpu.getAccZ());
  
//    netPrintf(F("GYRO      X: "));Serial.print(mpu.getGyroX());
//    netPrintf("\tY: ");Serial.print(mpu.getGyroY());
//    netPrintf("\tZ: ");netPrintf(mpu.getGyroZ());
  
//    netPrintf(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
//    netPrintf("\tY: ");netPrintf(mpu.getAccAngleY());
    
//    netPrintf(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
//    netPrintf("\tY: ");Serial.print(mpu.getAngleY());
//    netPrintf("\tZ: ");netPrintf(mpu.getAngleZ());
//    netPrintfln(F("=====================================================\n"));
//     timer = millis();
//   }