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

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include "PinConfig.h"

// === UDP 配置 ===
unsigned int localPort = 4210;      // 本地监听端口
WiFiUDP udp;
const int packetSize = 2;           // 每次收 2 字节：[pan, tilt]

// === 舵机引脚 ===
const int PAN_PIN = SERVO_PAN;
const int TILT_PIN = SERVO_TILT;
Servo panServo;
Servo tiltServo;

void Servo_init() {
    

    // 初始化舵机
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    panServo.setPeriodHertz(50);
    tiltServo.setPeriodHertz(50);
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    panServo.write(90);
    tiltServo.write(90);

    // 启动 UDP
    udp.begin(localPort);
    
}

void Servo_run() {
    int packetLen = udp.parsePacket();
    if (packetLen == packetSize) {
        uint8_t buffer[2];
        udp.read(buffer, packetSize);

        int pan = buffer[0];   // 第一个字节：0~255 → 我们只用 0~180
        int tilt = buffer[1];  // 第二个字节

        // 安全限制
        if (pan > 180) pan = 180;
        if (tilt > 180) tilt = 180;

        panServo.write(pan);
        tiltServo.write(tilt);

        // 打印调试信息
        Serial.print("UDP Received: pan=");
        Serial.print(pan);
        Serial.print(", tilt=");
        Serial.println(tilt);

       
    }

    delay(10); // 避免 CPU 占用 100%
}