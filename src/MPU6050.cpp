/*
 * Copyright 2026 徐畅
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
#include "PinConfig.h"
#include "SelfCheck.h"   // beepBuzzer
MPU6050 mpu(Wire);

// MPU6050_Init() 的探测结果，供里程计判断能否融合航向
static bool mpu_ready = false;


void MPU6050_Init() {
  
  Wire.begin(MPU_SDA, MPU_SCL);         //初始化I2C总线
  
  // ⚠️ 老代码这里是 while(status!=0){}：MPU 一旦读不到就整机静默卡死，
  //    表现为"车完全没自启也不报错"。改成有限重试 + 明确报警，不再死等。
  byte status = 0xFF;
  for (uint8_t i = 1; i <= 3; i++) {
    status = mpu.begin();
    if (status == 0) break;
    // Serial.printf("MPU6050 begin 失败(status=%u)，重试 %u/3\n", status, i);
    delay(100);
  }

  mpu_ready = (status == 0);

  if (status != 0) {
    Serial.println("❌ MPU6050 未检测到！继续启动（姿态数据无效，OLED 也可能不亮）");
    beepBuzzer(600);    // 长鸣一声提示，没有串口时也能听到
  }

  // ── 启用 I2C 旁路模式（Bypass）─────────────────────────
  // 设置 INT_PIN_CFG 寄存器(0x37) 的 bit 1 (BYPASS_EN = 0x02)
  // 使 MPU6050 将主 I2C 直通到 XDA/XCL，ESP32 可直接访问 OLED
  mpu.writeData(0x37, 0x02);
  Serial.println("I2C Bypass enabled (OLED via MPU XDA/XCL)");

  // MPU 没读到就别做偏移校准了：校准要跑上千次读数，每次都要等 I2C 超时，
  // 会白白卡住几十秒到几分钟
  if (status == 0) {
    Serial.print("Calculating offsets, do not move MPU6050");
    delay(1000);
    mpu.calcOffsets(true,true); // 校准角速度和加速度-------------------------------------------
    Serial.print("Done!\n");
  }
  
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

// ============================================================
// 更新 MPU 数据（每次读取前调用）
// ============================================================
void updateMPU() {
  mpu.update();
}

bool MPU6050_IsReady() {
  return mpu_ready;
}

// ============================================================
// 获取偏航角（Z 轴旋转，单位：度）
// ============================================================
float getMPUYaw() {
  return mpu.getAngleZ();
}

// ============================================================
// 获取俯仰角（X 轴，单位：度）
// ============================================================
float getMPUPitch() {
  return mpu.getAngleX();
}

// ============================================================
// 获取翻滚角（Y 轴，单位：度）
// ============================================================
float getMPURoll() {
  return mpu.getAngleY();
}

// ============================================================
// 陀螺仪数据（度/秒）
// ============================================================
float getMPUGyroX() { return mpu.getGyroX(); }
float getMPUGyroY() { return mpu.getGyroY(); }
float getMPUGyroZ() { return mpu.getGyroZ(); }

// ============================================================
// 加速度计数据（单位：g，1g = 9.81 m/s²）
// ============================================================
float getMPUAccelX() { return mpu.getAccX(); }
float getMPUAccelY() { return mpu.getAccY(); }
float getMPUAccelZ() { return mpu.getAccZ(); }