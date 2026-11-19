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

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H


// ──────────────────────────────────────────────────────────────
// 电机引脚（4 个直流电机）
// 每个电机 3 个引脚：IN1(方向A)、IN2(方向B)、PWM(速度)
// ──────────────────────────────────────────────────────────────
#define MOTOR1_IN1   7
#define MOTOR1_IN2   15
#define MOTOR1_PWM   16

#define MOTOR2_IN1   41
#define MOTOR2_IN2   40
#define MOTOR2_PWM   39

#define MOTOR3_IN1   13
#define MOTOR3_IN2   42
#define MOTOR3_PWM   1

#define MOTOR4_IN1   6
#define MOTOR4_IN2   5
#define MOTOR4_PWM   4


// ──────────────────────────────────────────────────────────────
// 编码器引脚（AB 相正交编码器）
// ──────────────────────────────────────────────────────────────
#define ENCODER0_A   3
#define ENCODER0_B   8

#define ENCODER1_A   48
#define ENCODER1_B   36

#define ENCODER2_A   38
#define ENCODER2_B   37

#define ENCODER3_A   17
#define ENCODER3_B   18


// ──────────────────────────────────────────────────────────────
// HC-SR04 超声波测距
// ──────────────────────────────────────────────────────────────
#define SONIC_TRIG   47
#define SONIC_ECHO   21

// ──────────────────────────────────────────────────────────────
// MPU6050 姿态传感器 (I2C)
// OLED 通过 MPU6050 的 XDA/XCL 旁路挂在同一 I2C 总线上
// ──────────────────────────────────────────────────────────────
#define MPU_SDA      9
#define MPU_SCL      14
#define OLED_SDA     MPU_SDA   // 与 MPU 共用同一 I2C 总线
#define OLED_SCL     MPU_SCL

// ──────────────────────────────────────────────────────────────
#define CFG_BUTTON   46
#define CFG_BUTTON_ACTIVE_LOW  1   // 1=按下为低(按键接地)；0=按下为高(按键接3V3)


// ──────────────────────────────────────────────────────────────
// 蜂鸣器
// ──────────────────────────────────────────────────────────────
#define BUZZER_PIN   12


// ──────────────────────────────────────────────────────────────
// 状态指示灯
// ──────────────────────────────────────────────────────────────
#define STUTS    2             // 数据接收指示灯（低电平闪烁）

// 电源指示灯：初始化成功后才置低电平点亮
#define POWER_LED       45
#define POWER_LED_ON    LOW    // 低电平点亮

// ──────────────────────────────────────────────────────────────
// 雷达 (YDLIDAR X2/X2L 兼容, TTL UART 150000 波特)
// ──────────────────────────────────────────────────────────────
#define LIDAR_TX   43   // ESP32 发送 (接雷达 RX)
#define LIDAR_RX   44   // ESP32 接收 (接雷达 TX)
#define LIDAR_BAUDRATE 150000

#endif // PIN_CONFIG_H


